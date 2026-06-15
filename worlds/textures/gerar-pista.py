"""Gera a textura da pista (pista.png) a partir do circuito desenhado pela equipe.

O traçado é um laço fechado (~95,90 x 58,66 cm) com um entalhe acentuado na parte
superior central, reproduzido como uma aproximação suave da silhueta original.
A linha tem 1,5 cm de espessura, conforme especificado.

A imagem é aplicada no chão da RectangleArena (2 x 2 m) como um único ladrilho
centralizado: o centro da imagem coincide com a origem do mundo, e os eixos batem
com os do Webots (x para a direita, y para cima).

Sem dependências externas: o PNG é escrito na mão com a biblioteca padrão
(zlib + struct). Rode com:  python3 gerar-pista.py
"""

import struct
import zlib

# --- Parâmetros geométricos ---------------------------------------------------
SIZE = 1024                # resolução da textura em pixels (quadrada)
ARENA_HALF_M = 1.0         # metade do lado da arena, em metros (arena = 2 x 2 m)
PX_POR_METRO = SIZE / (2 * ARENA_HALF_M)   # = 512 px/m
ESPESSURA_CM = 1.5         # espessura do NÚCLEO preto da linha, em centímetros
DESVANECE_CM = 0.8         # largura do halo que desvanece do preto ao branco em cada borda
#                            (dá ao sensor uma leitura ANALÓGICA na borda -> PID de verdade)

# Pontos de controle do EIXO da linha, em centímetros, com origem no centro
# (x para a direita, y para cima). Percorridos em sentido horário. A curva final
# é uma spline fechada de Catmull-Rom que passa suavemente por estes pontos.
PONTOS_CM = [
    (-48, 18),    # canto superior esquerdo
    (-40, 26),
    (-28, 28),    # topo do lobo esquerdo
    (-19, 24),    # começa a descer no vale
    (-13, 13),
    (-8, 2),
    (-4, -3),     # fundo do vale em U (largo e raso) - lado esquerdo
    (2, -3),      # fundo do vale em U - lado direito
    (7, 4),       # subindo do vale
    (12, 14),
    (18, 24),     # sobe para o lobo direito
    (28, 28),
    (38, 29),     # ponto mais alto (topo direito)
    (47, 26),
    (51, 14),     # canto superior direito
    (52, 0),      # lateral direita (ponto de partida do robô)
    (51, -14),    # canto inferior direito
    (44, -23),
    (33, -26),    # base do lobo direito
    (21, -24),
    (9, -19),     # subindo para a corcova inferior
    (0, -16),     # corcova inferior (rasa, bem abaixo do centro)
    (-10, -20),   # descendo
    (-21, -25),
    (-33, -26),   # base do lobo esquerdo
    (-43, -22),   # canto inferior esquerdo
    (-49, -4),    # lateral esquerda
]

PASSOS_POR_SEGMENTO = 80   # densidade de amostragem da spline

PRETO = 0
BRANCO = 255


def catmull_rom(p0, p1, p2, p3, t):
    """Ponto da spline de Catmull-Rom entre p1 e p2 (t em [0, 1])."""
    t2 = t * t
    t3 = t2 * t
    x = 0.5 * (
        (2 * p1[0])
        + (-p0[0] + p2[0]) * t
        + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2
        + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3
    )
    y = 0.5 * (
        (2 * p1[1])
        + (-p0[1] + p2[1]) * t
        + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2
        + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3
    )
    return x, y


def amostra_eixo():
    """Gera os pontos do eixo da linha percorrendo a spline fechada."""
    n = len(PONTOS_CM)
    pontos = []
    for i in range(n):
        p0 = PONTOS_CM[(i - 1) % n]
        p1 = PONTOS_CM[i]
        p2 = PONTOS_CM[(i + 1) % n]
        p3 = PONTOS_CM[(i + 2) % n]
        for s in range(PASSOS_POR_SEGMENTO):
            pontos.append(catmull_rom(p0, p1, p2, p3, s / PASSOS_POR_SEGMENTO))
    return pontos


def cm_para_px(x_cm, y_cm):
    """Converte (cm, centro na origem) para (coluna, linha) na imagem."""
    ix = round(SIZE / 2 + (x_cm / 100.0) * PX_POR_METRO)
    iy = round(SIZE / 2 - (y_cm / 100.0) * PX_POR_METRO)  # imagem cresce para baixo
    return ix, iy


def desenha():
    """Pinta o fundo branco e carimba a linha (núcleo preto + halo em gradiente).

    O carimbo é um disco cujo valor cresce do preto (centro) ao branco (borda do
    halo). Sobrepondo os discos ao longo do eixo e mantendo sempre o pixel mais
    ESCURO, a linha ganha um núcleo preto sólido com bordas que desvanecem.
    """
    linhas = [bytearray([BRANCO]) * (SIZE * 3) for _ in range(SIZE)]

    nucleo_px = (ESPESSURA_CM / 2) * PX_POR_METRO / 100.0
    halo_px = DESVANECE_CM * PX_POR_METRO / 100.0
    raio = max(1, round(nucleo_px + halo_px))

    # Pré-calcula o disco-carimbo com o valor (tom de cinza) de cada pixel.
    disco = []
    for dy in range(-raio, raio + 1):
        for dx in range(-raio, raio + 1):
            r = (dx * dx + dy * dy) ** 0.5
            if r <= nucleo_px:
                valor = PRETO
            elif r <= nucleo_px + halo_px:
                valor = round((r - nucleo_px) / halo_px * BRANCO)
            else:
                continue
            disco.append((dx, dy, valor))

    for x_cm, y_cm in amostra_eixo():
        cx, cy = cm_para_px(x_cm, y_cm)
        for dx, dy, valor in disco:
            x, y = cx + dx, cy + dy
            if 0 <= x < SIZE and 0 <= y < SIZE:
                base = x * 3
                if valor < linhas[y][base]:  # mantém o tom mais escuro (união dos discos)
                    linhas[y][base] = valor
                    linhas[y][base + 1] = valor
                    linhas[y][base + 2] = valor
    return linhas


def escreve_png(caminho, linhas):
    """Codifica as linhas RGB de 8 bits como PNG."""
    bruto = bytearray()
    for linha in linhas:
        bruto.append(0)  # byte de filtro (0 = nenhum)
        bruto += linha
    comprimido = zlib.compress(bytes(bruto), 9)

    def chunk(tipo, dados):
        corpo = tipo + dados
        return (
            struct.pack(">I", len(dados))
            + corpo
            + struct.pack(">I", zlib.crc32(corpo) & 0xFFFFFFFF)
        )

    assinatura = b"\x89PNG\r\n\x1a\n"
    ihdr = struct.pack(">IIBBBBB", SIZE, SIZE, 8, 2, 0, 0, 0)
    with open(caminho, "wb") as arquivo:
        arquivo.write(assinatura)
        arquivo.write(chunk(b"IHDR", ihdr))
        arquivo.write(chunk(b"IDAT", comprimido))
        arquivo.write(chunk(b"IEND", b""))


if __name__ == "__main__":
    escreve_png("pista.png", desenha())
    print(f"pista.png gerada ({SIZE}x{SIZE}px, linha de {ESPESSURA_CM} cm).")
