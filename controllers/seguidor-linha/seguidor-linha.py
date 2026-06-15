r"""Seguidor de BORDA com controle PID, para o LEGO Mindstorms EV3 (MindstormsRover).

================================================================================
IDEIA GERAL
================================================================================
O robô tem UM sensor de chão (S2) e deve manter esse sensor exatamente sobre a
BORDA da linha (a fronteira preto/branco). Em vez de decidir "preto/branco" no
liga-desliga (bang-bang), medimos QUÃO LONGE da borda estamos e corrigimos de
forma proporcional a esse erro -> trajetória mais suave, menos zigue-zague.

Leitura do sensor S2 (DistanceSensor infravermelho do PROTO), MEDIDA na simulação:
    PRETO  -> valor ALTO  (~62)
    BRANCO -> valor BAIXO (~21)
A borda da linha tem um halo em gradiente (ver gerar-pista.py), então entre as
duas cores o sensor lê valores intermediários: é essa rampa que dá ao termo P uma
faixa de atuação contínua (em vez de um degrau preto/branco). O alvo no meio dessa
rampa chama-se SETPOINT.

================================================================================
O CONTROLADOR PID
================================================================================
Definimos o erro como:

    e(t) = leitura(t) - SETPOINT

    e > 0  -> sensor entrou no preto  -> precisa virar para o branco (à direita)
    e < 0  -> sensor caiu no branco   -> precisa virar para o preto  (à esquerda)

A ação de controle (quanto e para que lado esterçar) é a soma de três termos:

    u(t) = Kp.e(t)  +  Ki.∫e(τ)dτ  +  Kd.de(t)/dt
           \_____/     \________/      \________/
              P            I               D
       (erro atual)  (erro acumulado)  (tendência do erro)

  - P (Proporcional): reage ao erro AGORA. Sozinho, deixa oscilação residual.
  - I (Integral): soma o histórico do erro; elimina desvio sistemático (viés),
        mas acumula demais perto da saturação ("windup") -> usamos anti-windup.
  - D (Derivativo): olha a VELOCIDADE do erro; antecipa e freia o overshoot,
        reduzindo o zigue-zague. É o termo mais útil aqui.

Como o controlador roda em passos discretos de duração Δt (o timestep), as
integrais/derivadas viram somas/diferenças (aproximação de Euler):

    integral   += e_k * Δt                  (área acumulada sob o erro)
    derivada    = (e_k - e_anterior) / Δt    (inclinação entre dois passos)
    u_k         = Kp*e_k + Ki*integral + Kd*derivada

A saída u vira correção diferencial nas rodas (esterçamento):

    roda_esq = BASE + u
    roda_dir = BASE - u

================================================================================
COMO EXPERIMENTAR (P, PI, PD, PID)
================================================================================
Basta zerar ganhos:

    P    ->  KI = 0.0   e  KD = 0.0
    PD   ->  KI = 0.0
    PI   ->  KD = 0.0
    PID  ->  os três diferentes de zero

Suba/desça um ganho de cada vez e observe: KP alto -> oscila; KD ajuda a
estabilizar; KI corrige viés mas pode "embalar" e instabilizar.
"""

from controller import Robot

# --- Inicialização ------------------------------------------------------------
robot = Robot()
timestep = int(robot.getBasicTimeStep())
dt = timestep / 1000.0  # passo de tempo em segundos (Δt)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
for motor in (left_motor, right_motor):
    motor.setPosition(float("inf"))  # modo de velocidade
    motor.setVelocity(0.0)

ground = robot.getDevice("S2")
ground.enable(timestep)

# --- Calibração do sensor / alvo ----------------------------------------------
# Valores MEDIDOS no console: branco ~21, preto ~62. O SETPOINT é a leitura-alvo
# na borda = ponto médio entre as duas cores: (21 + 62) / 2 ~= 41.
VALOR_BRANCO = 21.0
VALOR_PRETO = 62.0
SETPOINT = (VALOR_BRANCO + VALOR_PRETO) / 2.0

# --- Ganhos do PID (EDITE AQUI para comparar P / PI / PD / PID) ----------------
# Dica de ajuste: para deixar X vezes mais rápido, multiplique BASE, KP,
# MAX_SPEED e MIN_SPEED por X e MANTENHA o KD. O derivativo NÃO escala junto: ele
# já cresce sozinho com a velocidade (de/dt aumenta na mesma proporção); mexer no
# KD também faria amortecer demais. Assim o raio das curvas se mantém.
KP = 0.21  # proporcional
KI = 0.0  # integral    (deixe 0.0 para P ou PD)
KD = 0.003  # derivativo  (deixe 0.0 para P ou PI)

# --- Limites e velocidade base ------------------------------------------------
BASE = 3.9  # velocidade de cruzeiro das duas rodas (rad/s); o motor aceita até 10
MAX_SPEED = 9.0  # teto de velocidade de cada roda (rad/s)
MIN_SPEED = -1.8  # piso: ré permitida p/ fechar curvas, mas sem pivotar parado
INTEGRAL_MAX = 300.0  # anti-windup: limita o quanto o termo integral pode acumular


def clamp(valor, minimo, maximo):
    return max(minimo, min(maximo, valor))


# --- Estado do controlador (memória entre passos) -----------------------------
integral = 0.0
erro_anterior = 0.0

# --- Laço de controle ---------------------------------------------------------
while robot.step(timestep) != -1:
    leitura = ground.getValue()

    # 1) Erro atual
    erro = leitura - SETPOINT

    # 2) Termo Proporcional
    p_termo = KP * erro

    # 3) Termo Integral (soma de Riemann) com anti-windup
    integral += erro * dt
    integral = clamp(integral, -INTEGRAL_MAX, INTEGRAL_MAX)
    i_termo = KI * integral

    # 4) Termo Derivativo (diferença finita)
    derivada = (erro - erro_anterior) / dt
    d_termo = KD * derivada
    erro_anterior = erro

    # 5) Ação de controle = P + I + D
    u = p_termo + i_termo + d_termo

    # 6) Converte em velocidades diferenciais e satura
    left_speed = clamp(BASE + u, MIN_SPEED, MAX_SPEED)
    right_speed = clamp(BASE - u, MIN_SPEED, MAX_SPEED)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # Para calibrar (SETPOINT/ganhos), descomente e observe os valores no console:
    print(
        f"S2={leitura:6.1f}  e={erro:7.1f}  P={p_termo:6.3f}  I={i_termo:6.3f}  D={d_termo:6.3f}  u={u:6.3f}"
    )
