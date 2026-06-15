r"""Gera os gráficos de resposta do seguidor de borda a partir do PID ATUAL.

O script NÃO precisa do Webots. Ele lê os ganhos e parâmetros diretamente dos
arquivos do projeto (única fonte de verdade) e simula o laço fechado:

    controlador PID  ->  rodas (velocidade diferencial)  ->  cinemática do robô
        ^                                                         |
        |_______________ sensor de chão (S2) na borda  <_________|

Assim, sempre que você editar os ganhos no controlador, basta rodar este script
de novo para ver como a resposta muda. Rode com:

    python3 analise/gerar-graficos.py

Modelo da planta (cinemática de uniciclo, ângulos pequenos):
    leitura(t) = SETPOINT - S * y_sensor              (sensor na rampa da borda)
    e(t)       = leitura - SETPOINT
    u(t)       = Kp*e + Ki*∫e dt + Kd*de/dt           (o MESMO PID do controlador)
    v_esq/dir  = BASE ± u   (saturadas)  ->  v, ω
    dθ/dt      = ω = (v_esq - v_dir)*R_roda / entre_eixos
    dy_c/dt    = v * sin(θ)
    y_sensor   = y_c + look_ahead * sin(θ)            (sensor fica à frente do eixo)

O "degrau" é o robô começar deslocado lateralmente da linha (distúrbio inicial);
os gráficos mostram como ele retorna à borda.
"""

import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # backend sem janela: salva direto em PNG
import matplotlib.pyplot as plt
import numpy as np

# ==============================================================================
# 1) Ler os parâmetros ATUAIS dos arquivos do projeto
# ==============================================================================
RAIZ = Path(__file__).resolve().parent.parent
ARQ_CTRL = RAIZ / "controllers" / "seguidor-linha" / "seguidor-linha.py"
ARQ_MUNDO = RAIZ / "worlds" / "controle.wbt"
ARQ_PISTA = RAIZ / "worlds" / "textures" / "gerar-pista.py"
SAIDA = Path(__file__).resolve().parent / "resposta-pid.png"


def ler_constante(arquivo, nome, padrao):
    """Lê `nome = valor` (controlador/pista) ou `nome valor` (.wbt) de um arquivo."""
    texto = Path(arquivo).read_text(encoding="utf-8")
    achado = re.search(rf"^\s*{nome}\s*=?\s*([-+0-9.eE]+)", texto, re.MULTILINE)
    return float(achado.group(1)) if achado else padrao


# Ganhos e limites do controlador
KP = ler_constante(ARQ_CTRL, "KP", 0.21)
KI = ler_constante(ARQ_CTRL, "KI", 0.0)
KD = ler_constante(ARQ_CTRL, "KD", 0.003)
BASE = ler_constante(ARQ_CTRL, "BASE", 3.9)
MAX_SPEED = ler_constante(ARQ_CTRL, "MAX_SPEED", 9.0)
MIN_SPEED = ler_constante(ARQ_CTRL, "MIN_SPEED", -1.8)
INTEGRAL_MAX = ler_constante(ARQ_CTRL, "INTEGRAL_MAX", 300.0)
VALOR_BRANCO = ler_constante(ARQ_CTRL, "VALOR_BRANCO", 21.0)
VALOR_PRETO = ler_constante(ARQ_CTRL, "VALOR_PRETO", 62.0)
SETPOINT = (VALOR_BRANCO + VALOR_PRETO) / 2.0

# Passo de tempo (do mundo) e largura da rampa do sensor (da pista)
BASIC_TIME_STEP = ler_constante(ARQ_MUNDO, "basicTimeStep", 32.0)
DT = BASIC_TIME_STEP / 1000.0
DESVANECE_CM = ler_constante(ARQ_PISTA, "DESVANECE_CM", 0.8)

# ==============================================================================
# 2) Parâmetros físicos do robô (medidos no PROTO MindstormsRover)
# ==============================================================================
R_RODA = 0.01          # m  - raio da roda
ENTRE_EIXOS = 0.0454   # m  - distância entre as rodas (2 x 0.0227)
LOOK_AHEAD = 0.0146    # m  - sensor S2 à frente do centro do robô
LARGURA_RAMPA = DESVANECE_CM / 100.0  # m - faixa analógica do sensor na borda

# Ganho do sensor: quanto a leitura cai por metro de deslocamento lateral.
S_SENSOR = (VALOR_PRETO - VALOR_BRANCO) / LARGURA_RAMPA

# Distúrbio inicial: deslocamento lateral do robô em relação à borda.
DEGRAU_MM = 12.0
T_TOTAL = 4.0  # s de simulação


# ==============================================================================
# 3) Simulação do laço fechado
# ==============================================================================
def simular(kp, ki, kd, degrau_m=DEGRAU_MM / 1000.0, t_total=T_TOTAL):
    """Integra a planta + PID e devolve os históricos no tempo."""
    n = int(t_total / DT)
    y_c = degrau_m   # deslocamento lateral do centro (condição inicial = degrau)
    theta = 0.0
    integral = 0.0
    erro_ant = 0.0

    hist = {k: np.empty(n) for k in ("t", "e", "p", "i", "d", "u", "y_sensor")}

    for k in range(n):
        # --- sensor na borda (com saturação preto/branco) ---
        y_sensor = y_c + LOOK_AHEAD * np.sin(theta)
        leitura = SETPOINT - S_SENSOR * y_sensor
        leitura = min(VALOR_PRETO, max(VALOR_BRANCO, leitura))
        erro = leitura - SETPOINT

        # --- PID (idêntico ao controlador) ---
        p = kp * erro
        integral += erro * DT
        integral = min(INTEGRAL_MAX, max(-INTEGRAL_MAX, integral))
        i = ki * integral
        d = kd * (erro - erro_ant) / DT
        erro_ant = erro
        u = p + i + d

        # --- rodas e cinemática ---
        v_esq = min(MAX_SPEED, max(MIN_SPEED, BASE + u))
        v_dir = min(MAX_SPEED, max(MIN_SPEED, BASE - u))
        v = (v_esq + v_dir) / 2.0 * R_RODA
        omega = (v_esq - v_dir) * R_RODA / ENTRE_EIXOS

        theta += omega * DT
        y_c += v * np.sin(theta) * DT

        for nome, valor in (
            ("t", k * DT), ("e", erro), ("p", p), ("i", i),
            ("d", d), ("u", u), ("y_sensor", y_sensor * 1000.0),  # mm
        ):
            hist[nome][k] = valor

    return hist


# ==============================================================================
# 4) Métricas de desempenho da resposta ao degrau
# ==============================================================================
def metricas(t, y, valor_inicial, valor_final=0.0):
    """Overshoot (%), tempo de subida (10-90%), de pico e de acomodação (2%).

    A resposta parte de `valor_inicial` (o degrau) e deve assentar em
    `valor_final` (a borda, = 0). Overshoot é a excursão ALÉM do alvo.
    """
    t = np.asarray(t)
    y = np.asarray(y)
    delta = valor_inicial - valor_final  # amplitude do degrau (positiva aqui)

    # Pico e overshoot (no nosso caso o sinal cai e ultrapassa o alvo por baixo)
    if delta >= 0:
        pico = float(y.min())
        idx_pico = int(np.argmin(y))
        overshoot = max(0.0, valor_final - pico) / abs(delta) * 100.0
    else:
        pico = float(y.max())
        idx_pico = int(np.argmax(y))
        overshoot = max(0.0, pico - valor_final) / abs(delta) * 100.0
    t_pico = float(t[idx_pico])

    # Tempo de subida: 90% -> 10% da variação (transitório principal)
    def primeiro_instante(nivel, descendo):
        cond = (y <= nivel) if descendo else (y >= nivel)
        idx = np.argmax(cond)
        return float(t[idx]) if cond[idx] else float("nan")

    nivel_90 = valor_final + 0.9 * delta
    nivel_10 = valor_final + 0.1 * delta
    t_subida = primeiro_instante(nivel_10, delta >= 0) - primeiro_instante(
        nivel_90, delta >= 0
    )

    # Tempo de acomodação: último instante fora da faixa de 2% do degrau
    tol = 0.02 * abs(delta)
    fora = np.where(np.abs(y - valor_final) > tol)[0]
    t_acomod = float(t[fora[-1] + 1]) if len(fora) and fora[-1] + 1 < len(t) else 0.0

    return {
        "overshoot": overshoot,
        "t_pico": t_pico,
        "t_subida": t_subida,
        "t_acomod": t_acomod,
        "pico": pico,
        "tol": tol,
    }


# ==============================================================================
# 5) Gráficos
# ==============================================================================
def main():
    atual = simular(KP, KI, KD)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))
    fig.suptitle(
        f"Resposta do seguidor de borda — PID atual\n"
        f"KP={KP:g}  KI={KI:g}  KD={KD:g}  |  BASE={BASE:g}  "
        f"SETPOINT={SETPOINT:g}  Δt={DT*1000:g} ms  degrau={DEGRAU_MM:g} mm",
        fontsize=12,
    )

    # (a) Resposta ao degrau: posição lateral do sensor voltando à borda
    m = metricas(atual["t"], atual["y_sensor"], DEGRAU_MM, 0.0)
    ax1.axhspan(-m["tol"], m["tol"], color="C2", alpha=0.12, label="faixa 2%")
    ax1.axhline(0, color="gray", lw=1, ls="--", label="borda (alvo)")
    ax1.plot(atual["t"], atual["y_sensor"], color="C0", lw=2)
    ax1.plot(m["t_pico"], m["pico"], "v", color="C3", ms=9, label="pico (overshoot)")
    ax1.axvline(m["t_acomod"], color="C2", ls=":", lw=1.5)
    ax1.set_title("(a) Resposta ao degrau — distância do sensor à borda")
    ax1.set_xlabel("tempo (s)")
    ax1.set_ylabel("y do sensor (mm)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="lower right")
    texto = (
        f"Overshoot (Mp): {m['overshoot']:.1f} %\n"
        f"t. subida (10-90%): {m['t_subida']:.2f} s\n"
        f"t. pico (tp): {m['t_pico']:.2f} s\n"
        f"t. acomodação (2%): {m['t_acomod']:.2f} s"
    )
    ax1.text(
        0.97, 0.95, texto, transform=ax1.transAxes, ha="right", va="top", fontsize=9,
        family="monospace", bbox=dict(boxstyle="round", fc="white", ec="0.6", alpha=0.9),
    )

    # (b) Contribuição de cada termo do PID e a ação de controle u
    ax2.plot(atual["t"], atual["p"], label="P", color="C1")
    ax2.plot(atual["t"], atual["i"], label="I", color="C2")
    ax2.plot(atual["t"], atual["d"], label="D", color="C3")
    ax2.plot(atual["t"], atual["u"], label="u = P+I+D", color="k", lw=2)
    ax2.set_title("(b) Termos do PID e ação de controle (esterçamento)")
    ax2.set_xlabel("tempo (s)")
    ax2.set_ylabel("contribuição (rad/s)")
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc="upper right", ncol=4)

    # (c) Comparação P / PD / PI / PID (mesma resposta ao degrau)
    # Se o KI atual for 0, usa um KI ilustrativo só para as curvas com integral,
    # para que o efeito do termo I apareça no gráfico.
    ki_demo = KI if KI > 0 else 0.02 * KP
    variantes = [
        ("P", KP, 0.0, 0.0, "C1"),
        ("PD", KP, 0.0, KD, "C0"),
        ("PI", KP, ki_demo, 0.0, "C2"),
        ("PID", KP, ki_demo, KD, "C3"),
    ]
    ax3.axhline(0, color="gray", lw=1, ls="--")
    for nome, kp, ki, kd, cor in variantes:
        h = simular(kp, ki, kd)
        mv = metricas(h["t"], h["y_sensor"], DEGRAU_MM, 0.0)
        rotulo = f"{nome}  (Mp={mv['overshoot']:.0f}%, ts={mv['t_acomod']:.2f}s)"
        ax3.plot(h["t"], h["y_sensor"], label=rotulo, color=cor, lw=1.8)
    nota_ki = "" if KI > 0 else f"  (KI ilustrativo={ki_demo:g} nas curvas com I)"
    ax3.set_title("(c) Comparação P / PD / PI / PID" + nota_ki)
    ax3.set_xlabel("tempo (s)")
    ax3.set_ylabel("y do sensor (mm)")
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc="upper right", ncol=4)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(SAIDA, dpi=120)
    print(f"Gráfico salvo em: {SAIDA}")
    print(
        f"Ganhos lidos -> KP={KP:g}  KI={KI:g}  KD={KD:g}  "
        f"BASE={BASE:g}  SETPOINT={SETPOINT:g}  Δt={DT*1000:g} ms"
    )


if __name__ == "__main__":
    main()
