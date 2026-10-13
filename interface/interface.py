import sys
from pathlib import Path
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QFormLayout, QGroupBox, QDoubleSpinBox, QComboBox,
    QPushButton, QLabel, QMessageBox, QFileDialog, QCheckBox)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import scipy.io as sio
import control as ctrl

K = 17.951609279384616
TAU = 7.776452028082398
THETA = 0.9466405157382338

# Calcula os parâmetros de sintonia CHR ou ITAE.
def calcular_sintonia(metodo):
    if metodo == 'CHR sem sobrevalor':
        return 0.6 * TAU / (K * THETA), TAU, THETA / 2
    if metodo == 'ITAE':
        r = THETA / TAU
        return (0.965 / K) * r ** (-0.85), TAU / (0.796 - 0.147 * r), TAU * 0.308 * r ** 0.929
    raise ValueError('Método não reconhecido')


# Monta o modelo FOPDT identificado com atraso aproximado por Padé.
def criar_planta():
    planta = ctrl.tf([K], [TAU, 1])
    numerador, denominador = ctrl.pade(THETA, 10)
    return ctrl.series(planta, ctrl.tf(numerador, denominador))


# Cria o controlador PID contínuo com filtro derivativo.
def criar_controlador(kp, ti, td):
    if kp < 0 or ti <= 0 or td < 0:
        raise ValueError('Use Kp >= 0, Ti > 0 e Td >= 0.')
    s = ctrl.tf('s')
    return ctrl.minreal(kp * (1 + 1 / (ti * s) + td * s / (1 + (td / 10) * s)), verbose=False)


# Simula a resposta do motor em malha fechada.
def simular(kp, ti, td, setpoint, duracao=40):
    controlador = criar_controlador(kp, ti, td)
    sistema = ctrl.feedback(ctrl.series(controlador, criar_planta()), 1)
    polos = ctrl.poles(sistema)
    if np.any(np.real(polos) >= 0):
        raise ValueError('A sintonia produz uma malha fechada instável.')
    tempo = np.linspace(0, duracao, int(duracao * 100) + 1)
    tempo, velocidade = ctrl.step_response(setpoint * sistema, tempo)
    velocidade = np.asarray(velocidade).ravel()
    if not np.all(np.isfinite(velocidade)):
        raise ValueError('A simulação produziu valores inválidos.')
    metricas = calcular_metricas(tempo, velocidade, setpoint, sistema)
    return tempo, velocidade, metricas


# Calcula as métricas pelo mesmo método usado no notebook.
def calcular_metricas(tempo, velocidade, setpoint, sistema):
    info = ctrl.step_info(setpoint * sistema, T=tempo, SettlingTimeThreshold=0.02)
    final = float(np.real(info['SteadyStateValue']))
    pico_indice = int(np.argmax(velocidade))
    pico = float(velocidade[pico_indice])
    tr = float(info['RiseTime'])
    ts = float(info['SettlingTime'])
    t10 = None
    t90 = None
    if final > 0:
        for chave, fracao in [('t10', 0.1), ('t90', 0.9)]:
            indices = np.where(velocidade >= fracao * final)[0]
            if len(indices):
                i = int(indices[0])
                instante = float(tempo[0]) if i == 0 else float(np.interp(fracao * final, velocidade[i-1:i+1], tempo[i-1:i+1]))
                if chave == 't10':
                    t10 = instante
                else:
                    t90 = instante
    return {'final': final, 'pico': pico, 'erro': setpoint - final,
            'tr': tr if np.isfinite(tr) else None,
            'ts': ts if np.isfinite(ts) and ts <= tempo[-1] else None,
            't10': t10, 't90': t90,
            'overshoot': float(info['Overshoot']) if np.isfinite(info['Overshoot']) else 0.0}


# Simula a resposta do modelo identificado ao degrau experimental.
def simular_identificacao(tempo, degrau, saida):
    mudancas = np.where(np.abs(np.diff(degrau)) > 1e-9)[0]
    if not len(mudancas):
        raise ValueError('O dataset não contém um degrau identificável.')
    indice = int(mudancas[0] + 1)
    t0 = float(tempo[indice])
    amplitude = float(degrau[-1] - degrau[0])
    base = float(np.mean(saida[:indice])) if indice > 0 else float(saida[0])
    tempo_relativo = np.asarray(tempo, dtype=float) - t0
    t_modelo = np.linspace(0, max(0.01, float(tempo_relativo[-1])), 2000)
    t_modelo, resposta = ctrl.step_response(amplitude * criar_planta(), T=t_modelo)
    return tempo_relativo, np.asarray(t_modelo), np.asarray(resposta).ravel() + base


# Carrega as amostras experimentais do arquivo MAT.
def carregar_dataset(caminho):
    dados = sio.loadmat(str(Path(caminho)))
    return (np.asarray(dados['t']).ravel(),
            np.asarray(dados['Degrau']).ravel(),
            np.asarray(dados['Saida']).ravel())



class Janela(QMainWindow):
    # Inicializa a janela e os componentes da aplicação.
    def __init__(self):
        super().__init__()
        self.setWindowTitle('C13 | Grupo 6 - Controle PID do Motor CC')
        self.resize(1120, 720)
        self.fig = Figure(figsize=(8, 5))
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.criar_interface()
        self.atualizar_modo()

    # Organiza os controles e o gráfico em uma única tela.
    def criar_interface(self):
        central = QWidget()
        self.setCentralWidget(central)
        horizontal = QHBoxLayout(central)
        painel = QVBoxLayout()
        horizontal.addLayout(painel, 1)
        horizontal.addWidget(self.canvas, 3)
        titulo = QLabel('Controle de velocidade — Motor CC | Grupo 6')
        titulo.setStyleSheet('font-size: 16px; font-weight: bold;')
        painel.addWidget(titulo)
        self.criar_controles(painel)

    # Cria os campos de sintonia e os botões de ação.
    def criar_controles(self, painel):
        grupo = QGroupBox('Parâmetros do controlador')
        formulario = QFormLayout(grupo)
        self.modo = QComboBox()
        self.modo.addItems(['CHR sem sobrevalor', 'ITAE', 'Manual'])
        self.modo.currentTextChanged.connect(self.atualizar_modo)
        formulario.addRow('Sintonia:', self.modo)
        self.sp = self.criar_spin(0, 3000, 900, 1)
        formulario.addRow('SetPoint (RPM):', self.sp)
        self.kp = self.criar_spin(0, 10000, 0.3, 0.001)
        self.ti = self.criar_spin(0.0001, 10000, 8, 0.001)
        self.td = self.criar_spin(0, 10000, 0.4, 0.001)
        for nome, campo in [('Kp:', self.kp), ('Ti (s):', self.ti), ('Td (s):', self.td)]:
            formulario.addRow(nome, campo)
        painel.addWidget(grupo)
        self.comparar = QCheckBox('Comparar CHR e ITAE')
        self.comparar.toggled.connect(self.simular_resposta)
        painel.addWidget(self.comparar)
        executar = QPushButton('Simular resposta')
        executar.clicked.connect(self.simular_resposta)
        painel.addWidget(executar)
        exportar = QPushButton('Exportar gráfico PNG')
        exportar.clicked.connect(self.exportar)
        painel.addWidget(exportar)
        grupo_dados = QGroupBox('Planta identificada — Sundaresan')
        dados_layout = QVBoxLayout(grupo_dados)
        dados_layout.addWidget(QLabel(f'k = {K:.4f} RPM/% | τ = {TAU:.4f} s | θ = {THETA:.4f} s'))
        botao_mat = QPushButton('Carregar e visualizar dataset (.mat)')
        botao_mat.clicked.connect(self.abrir_dataset)
        dados_layout.addWidget(botao_mat)
        painel.addWidget(grupo_dados)
        self.aviso = QLabel('Modelo linear sem saturação de PWM.')
        self.aviso.setWordWrap(True)
        painel.addWidget(self.aviso)
        self.metricas = QLabel()
        self.metricas.setWordWrap(True)
        painel.addWidget(self.metricas)
        painel.addStretch()

    # Cria um campo numérico com os limites informados.
    def criar_spin(self, minimo, maximo, valor, passo):
        campo = QDoubleSpinBox()
        campo.setRange(minimo, maximo)
        campo.setDecimals(6)
        campo.setSingleStep(passo)
        campo.setValue(valor)
        return campo

    # Atualiza os campos conforme o modo de sintonia.
    def atualizar_modo(self):
        manual = self.modo.currentText() == 'Manual'
        if not manual:
            kp, ti, td = calcular_sintonia(self.modo.currentText())
            for campo, valor in [(self.kp, kp), (self.ti, ti), (self.td, td)]:
                campo.setValue(valor)
        for campo in (self.kp, self.ti, self.td):
            campo.setReadOnly(not manual)
            campo.setButtonSymbols(QDoubleSpinBox.UpDownArrows if manual else QDoubleSpinBox.NoButtons)
        self.simular_resposta()

    # Executa a simulação selecionada e mostra os resultados.
    def simular_resposta(self):
        try:
            sp = self.sp.value()
            if sp <= 0:
                raise ValueError('Informe um SetPoint maior que zero.')
            if self.comparar.isChecked():
                casos = [(nome, *calcular_sintonia(nome)) for nome in ('CHR sem sobrevalor', 'ITAE')]
            else:
                casos = [(self.modo.currentText(), self.kp.value(), self.ti.value(), self.td.value())]
            resultados = [(nome, *simular(kp, ti, td, sp)) for nome, kp, ti, td in casos]
            self.atualizar_grafico(resultados, sp)
            self.atualizar_metricas(resultados)
            self.aviso.setText('Modelo linear sem saturação de PWM.')
        except Exception as erro:
            self.metricas.setText(f'Erro: {erro}')
            QMessageBox.warning(self, 'Simulação', str(erro))

    # Desenha a resposta e marca os pontos de interesse.
    def atualizar_grafico(self, resultados, sp):
        self.fig.clear()
        ax = self.fig.add_subplot(111)
        ax.axhline(sp, linestyle='--', color='black', label=f'SetPoint: {sp:g} RPM')
        ax.axhspan(sp * 0.98, sp * 1.02, color='gray', alpha=0.12, label='±2% do SetPoint')
        cores = ['tab:blue', 'tab:red']
        for i, (nome, tempo, velocidade, dados) in enumerate(resultados):
            cor = cores[i % len(cores)]
            ax.plot(tempo, velocidade, color=cor, label=nome)
            for chave, legenda in [('t10', '10%'), ('t90', '90%'), ('ts', 'ts')]:
                instante = dados[chave]
                if instante is not None:
                    altura = float(np.interp(instante, tempo, velocidade))
                    ax.plot(instante, altura, 'o', color=cor, markersize=5)
                    ax.annotate(f'{legenda}: {instante:.2f}s', (instante, altura),
                                xytext=(6, 10 + 12 * i), textcoords='offset points', fontsize=8)
            pico_idx = int(np.argmax(velocidade))
            ax.plot(tempo[pico_idx], velocidade[pico_idx], 'x', color=cor, markersize=8)
            ax.annotate(f'Pico: {dados["pico"]:.1f}', (tempo[pico_idx], velocidade[pico_idx]),
                        xytext=(6, -18 - 12 * i), textcoords='offset points', fontsize=8)
        ax.set(xlabel='Tempo (s)', ylabel='Velocidade (RPM)', title='Resposta em malha fechada — PID')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=8)
        self.fig.tight_layout()
        self.canvas.draw()

    # Apresenta as métricas calculadas de cada controlador.
    def atualizar_metricas(self, resultados):
        linhas = []
        for nome, _, _, dados in resultados:
            tr = f'{dados["tr"]:.2f} s' if dados['tr'] is not None else 'Não atingido'
            ts = f'{dados["ts"]:.2f} s' if dados['ts'] is not None else 'Não acomodou em 40 s'
            linhas.append(f'{nome}\ntr: {tr} | ts: {ts}\nPico: {dados["pico"]:.2f} RPM'
                          f' | Overshoot: {dados["overshoot"]:.2f}%\n'
                          f'Erro em regime: {dados["erro"]:.3f} RPM')
        self.metricas.setText('\n\n'.join(linhas))

    # Abre um arquivo MAT escolhido pelo usuário.
    def abrir_dataset(self):
        caminho, _ = QFileDialog.getOpenFileName(self, 'Selecionar dataset', '', 'MAT (*.mat)')
        if caminho:
            self.mostrar_dataset(caminho)

    # Mostra os dados experimentais e o modelo no gráfico principal.
    def mostrar_dataset(self, caminho):
        try:
            tempo, degrau, saida = carregar_dataset(caminho)
            tempo_rel, tempo_modelo, resposta = simular_identificacao(tempo, degrau, saida)
            self.fig.clear()
            eixo = self.fig.add_subplot(111)
            eixo.plot(tempo_rel, saida, '.', markersize=3, alpha=0.55, label='Dados experimentais')
            eixo.plot(tempo_modelo, resposta, linewidth=2, label='Modelo Sundaresan (FOPDT)')
            eixo.set(xlabel='Tempo relativo ao degrau (s)', ylabel='Velocidade (RPM)',
                     title='Identificação Sundaresan — Motor G6')
            eixo.grid(True, alpha=0.3)
            eixo.legend()
            self.fig.tight_layout()
            self.canvas.draw()
            self.aviso.setText('Visualizando identificação. Clique em Simular resposta para voltar ao PID.')
        except Exception as erro:
            QMessageBox.warning(self, 'Dataset', str(erro))

    # Salva o gráfico da simulação em PNG.
    def exportar(self):
        caminho, _ = QFileDialog.getSaveFileName(self, 'Salvar gráfico', 'pid_grupo6.png', 'PNG (*.png)')
        if caminho:
            if not caminho.lower().endswith('.png'):
                caminho += '.png'
            self.fig.savefig(caminho, dpi=200, bbox_inches='tight')
            QMessageBox.information(self, 'Exportar', 'Gráfico salvo com sucesso.')


# Inicia a aplicação gráfica.
def main():
    app = QApplication(sys.argv)
    janela = Janela()
    janela.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
