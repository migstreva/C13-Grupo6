import sys
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QFormLayout, QGroupBox, QDoubleSpinBox, QComboBox,
    QPushButton, QLabel, QFileDialog, QMessageBox,
)
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

GANHO_MOTOR = 18.0
TAU_MOTOR = 8.0
ATRASO_MOTOR = 0.8
TEMPO_SIMULACAO = 60.0
PASSO = 0.02


# Calcula os parâmetros PID para o método selecionado.
def calcular_pid(modo):
    k, tau, theta = GANHO_MOTOR, TAU_MOTOR, ATRASO_MOTOR

    if modo == 'CHR sem overshoot':
        return 0.6 * tau / (k * theta), tau, theta / 2

    if modo == 'ITAE':
        razao = theta / tau
        kp = (0.965 / k) * razao ** (-0.85)
        ti = tau / (0.796 - 0.147 * razao)
        td = tau * 0.308 * razao ** 0.929
        return kp, ti, td

    raise ValueError('Modo de sintonia desconhecido.')


# Simula a velocidade do motor com o controlador PID.
def simular_motor(kp, ti, td, setpoint):
    tempo = np.arange(0, TEMPO_SIMULACAO + PASSO, PASSO)
    velocidade = np.zeros(len(tempo))
    pwm = np.zeros(len(tempo))
    integral = 0.0
    velocidade_anterior = 0.0
    passos_atraso = round(ATRASO_MOTOR / PASSO)

    for i in range(1, len(tempo)):
        erro = setpoint - velocidade[i - 1]
        derivada = -(velocidade[i - 1] - velocidade_anterior) / PASSO
        comando = kp * (erro + integral + td * derivada)

        pode_integrar = (
            0 < comando < 100
            or (comando >= 100 and erro < 0)
            or (comando <= 0 and erro > 0)
        )
        if pode_integrar:
            integral += erro * PASSO / ti

        pwm[i] = np.clip(kp * (erro + integral + td * derivada), 0, 100)

        pwm_atrasado = pwm[i - passos_atraso] if i >= passos_atraso else 0.0
        velocidade[i] = velocidade[i - 1] + PASSO * (
            -velocidade[i - 1] + GANHO_MOTOR * pwm_atrasado
        ) / TAU_MOTOR
        velocidade_anterior = velocidade[i - 1]

    return tempo, velocidade, pwm


class Janela(QMainWindow):
    # Inicializa a janela principal.
    def __init__(self):
        super().__init__()
        self.setWindowTitle('C13 | Grupo 6 - Controle PID do Motor CC')
        self.resize(1050, 650)
        self.criar_interface()
        self.atualizar_modo()

    # Organiza os elementos da interface.
    def criar_interface(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)

        controles = self.criar_controles()
        layout.addLayout(controles, 1)

        self.criar_grafico()
        layout.addWidget(self.canvas, 3)

    # Cria um campo para valores numéricos.
    def criar_spinbox(self, minimo, maximo, inicial, passo):
        campo = QDoubleSpinBox()
        campo.setRange(minimo, maximo)
        campo.setValue(inicial)
        campo.setDecimals(4)
        campo.setSingleStep(passo)
        return campo

    # Cria os campos e botões de controle.
    def criar_controles(self):
        layout = QVBoxLayout()
        grupo = QGroupBox('Sintonia do controlador')
        formulario = QFormLayout(grupo)
        layout.addWidget(grupo)

        self.modo = QComboBox()
        self.modo.addItems(['Manual', 'CHR sem overshoot', 'ITAE'])
        formulario.addRow('Modo:', self.modo)

        self.sp = self.criar_spinbox(0, 3000, 900, 1)
        formulario.addRow('SetPoint (RPM):', self.sp)

        self.kp = self.criar_spinbox(0, 10000, 1, 0.001)
        self.ti = self.criar_spinbox(0.001, 10000, 1, 0.001)
        self.td = self.criar_spinbox(0, 10000, 0, 0.001)
        formulario.addRow('Kp:', self.kp)
        formulario.addRow('Ti (s):', self.ti)
        formulario.addRow('Td (s):', self.td)

        informacao = QLabel('Modelo: k=18 RPM/%, τ=8 s, θ=0,8 s.')
        informacao.setWordWrap(True)
        layout.addWidget(informacao)

        botao_simular = QPushButton('Simular resposta')
        botao_simular.clicked.connect(self.simular)
        layout.addWidget(botao_simular)

        botao_exportar = QPushButton('Exportar gráfico PNG')
        botao_exportar.clicked.connect(self.exportar)
        layout.addWidget(botao_exportar)

        self.metricas = QLabel('')
        self.metricas.setWordWrap(True)
        layout.addWidget(self.metricas)
        layout.addStretch()

        self.modo.currentIndexChanged.connect(self.atualizar_modo)
        return layout

    # Cria a área do gráfico.
    def criar_grafico(self):
        self.figura = Figure(figsize=(7, 5))
        self.canvas = FigureCanvasQTAgg(self.figura)

    # Configura os campos conforme o modo de sintonia.
    def atualizar_modo(self, *_):
        modo = self.modo.currentText()
        manual = modo == 'Manual'

        if not manual:
            kp, ti, td = calcular_pid(modo)
            self.kp.setValue(kp)
            self.ti.setValue(ti)
            self.td.setValue(td)

        for campo in (self.kp, self.ti, self.td):
            campo.setReadOnly(not manual)
            simbolos = (QDoubleSpinBox.UpDownArrows if manual
                        else QDoubleSpinBox.NoButtons)
            campo.setButtonSymbols(simbolos)

        self.simular()

    # Executa a simulação e atualiza os resultados.
    def simular(self):
        setpoint = self.sp.value()
        tempo, velocidade, pwm = simular_motor(
            self.kp.value(), self.ti.value(), self.td.value(), setpoint
        )
        self.atualizar_grafico(tempo, velocidade, setpoint)
        self.atualizar_metricas(velocidade, pwm, setpoint)

    # Desenha a velocidade e o SetPoint no gráfico.
    def atualizar_grafico(self, tempo, velocidade, setpoint):
        self.figura.clear()
        eixo = self.figura.add_subplot(111)
        eixo.plot(tempo, velocidade, label='Velocidade simulada (RPM)')
        eixo.axhline(setpoint, linestyle='--', label='SetPoint')
        eixo.set_xlabel('Tempo (s)')
        eixo.set_ylabel('Velocidade (RPM)')
        eixo.set_title('Resposta em malha fechada')
        eixo.grid(True, alpha=0.3)
        eixo.legend()
        self.figura.tight_layout()
        self.canvas.draw()

    # Mostra as métricas da simulação.
    def atualizar_metricas(self, velocidade, pwm, setpoint):
        pico = float(np.max(velocidade))
        overshoot = max(0.0, (pico - setpoint) / setpoint * 100) if setpoint > 0 else 0.0
        self.metricas.setText(
            f'Pico: {pico:.1f} RPM | Overshoot: {overshoot:.1f}% '
            f'| PWM final: {pwm[-1]:.1f}%'
        )

    # Salva o gráfico em uma imagem PNG.
    def exportar(self):
        caminho, _ = QFileDialog.getSaveFileName(
            self, 'Salvar gráfico', 'pid_grupo6.png', 'Imagens PNG (*.png)'
        )
        if caminho:
            self.figura.savefig(caminho, dpi=160)
            QMessageBox.information(self, 'Exportado', 'Gráfico salvo com sucesso.')


# Inicia a aplicação.
def main():
    app = QApplication(sys.argv)
    janela = Janela()
    janela.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
