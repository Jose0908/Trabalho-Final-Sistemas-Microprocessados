/*  Aluno: José Roberto Interaminense Soares
 *  Matrícula: 190130008
 *  Trabalho Final - Laboratório de Sistemas Microprocessados
 *  Todas as especificações se encontram no PDF
 */
#include <msp430.h> 
/* Acender e desligar leds */
#define LED1_ON         (P1OUT |= BIT0)
#define LED1_OFF        (P1OUT &= ~BIT0)
#define LED2_ON         (P4OUT |= BIT7)
#define LED2_OFF        (P4OUT &= ~BIT7)

#define UMIDADE_BAIXA_LIMITE      195       // Umidade medida empiricamente (valor proveniente do ADC)
#define CONVERSAO_EM_PORCENTAGEM  0.02442   // Usado no ADC: 4095 --> 100%,  conversao --> y%. 100/4095 = 0.02442

/* Funções Utilizadas */
void io_config();
void init_timer();
void init_ADC();
void init_UART();
void bem_vindo();
void status_Planta();
void regar_Planta();
/* Variaveis Globais */
unsigned int i;
int umidade;
int planta_foi_regada = 0;
int welcome = 1;
int s1_pressed = 0;
int umidade_baixa = 0;
/* Mensagens enviadas para o Bluetooth */
char regar[33] = {'A', ' ','P','L','A','N','T','A',' ','E','S','T','A',' ','T','R','I','S','T','E',' ',':','(',' ','R','E','G','U','E','-','A','\n'};
char nao_regar[33] = {'A', ' ','P','L','A','N','T','A',' ','E','S','T','A',' ','F','E','L','I','Z',' ',':',')',' ','P','A','R','A','B','E','N','S','\n'};
char porcentagem[9] = {'X','X','%','U','M','I','D','A','\n'};
char regue[15] = {'R','E','G','U','E',' ','A',' ','P','L','A','N','T','A','\n'};
char boas_vindas[16] = {'\n','B','O','T','A','O',' ','=',' ','S','T','A','T','U','S','\n'};

/**
 * main.c
 */
void main()
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    io_config();       // Configuração dos Leds, botão S1 e entrada digital do sensor de umidade
    init_timer();      // Configuração de 2 timers: TA0 para conversão ADC (1hz) e TA1 para enviar mensagem de regar planta (0.25 Hz)
    init_ADC();        // Converte a entrada analógica do sensor de umidade
    init_UART();       // Faz a comunicação com o módulo bluetoth
    __enable_interrupt();   // Habilita 3 interrupções: Botão S1, conversão ADC e timer (só é ligado quando é preciso regar planta)

    while (1)
    {
        if (welcome)                    // Caso haja necessidade de dar boas vindas (por padrão já é 1)
        {
            welcome = 0;                // Boas vindas imprimida apenas uma vez
            bem_vindo();
            planta_foi_regada = 0;
        }
        if (s1_pressed)
        {
            __delay_cycles(300000);     // Debounce
            s1_pressed = 0;
            status_Planta();            // Mostra uma vez o status da planta
        }


        if (umidade_baixa)              // Se umidade esta baixa: mandar mensagem de regar planta
        {
            planta_foi_regada = 1;      // Quando a umidade aumentar, a planta terá sido regada
            regar_Planta();
        } else
        {
            LED1_OFF;
            LED2_ON;
            TA1CTL |= TASSEL_1|MC__STOP;    // Timer parado
            TA1CCTL1 = 0;
            if (planta_foi_regada) {        // WELCOME
                welcome = 1;
            }
        }
    }
}
/* --------------   MODOS DE OPERACAO ---------------- */
/* Funçao que imprime mensagem de boas vindas */
void bem_vindo()
{
    for (i = 0; i < 16; i++) {
        UCA0TXBUF = boas_vindas[i];         // Imprime mensagem de boas vindas
        while (!(UCA0IFG&UCTXIFG));
    }
}
/* Funçao que imprime status (feliz e triste) da planta */
void status_Planta()
{
    int porcent = 0;

    porcent = (umidade*CONVERSAO_EM_PORCENTAGEM);                   // Calcula % de umidade
    porcentagem[0] = (porcent/10) + '0';
    porcentagem[1] = ((porcent - ((porcentagem[0]-48)*10)) + '0');

        for (i = 0; i < 32; i++) {                                  // Imprime status triste
            if (umidade_baixa)
                UCA0TXBUF = regar[i];
            else
                UCA0TXBUF = nao_regar[i];                           // Imprime status feliz
            while (!(UCA0IFG&UCTXIFG));
        }
        for (i = 0; i < 9; i++) {
            UCA0TXBUF = porcentagem[i];                             // Em ambos os casos (feliz e triste), imprime %
            while (!(UCA0IFG&UCTXIFG));
        }
}
/* Funçao que liga interrupção para enviar mensagens de "regue a planta" para usuário */
void regar_Planta()
{
    LED1_ON;
    LED2_OFF;
    TA1CTL |= MC__UP;
    TA1CCTL1 |= CCIE;
}

/*----------------------------- FUNÇÕES DE INICIALIZAÇÃO -------------------------------- */
/* Funçao para configurar LEDS, Botão S1,  Saída DIGITAL do sensor de umidade*/
void io_config()
{
    P1DIR |= BIT0;              // Led1 = P1.0 = saída
    P1OUT &= ~BIT0;             // Led1 apagado (configuração Default do programa)

    P4DIR |= BIT7;              // Led2 = P4.7 = saída
    P4OUT &= ~BIT7;             // Led2 apagado (configuração Default do programa)

    P2DIR &= ~BIT5;             // Entrada digital do sensor
    P2REN |= BIT5;              // Habilitar resistor
    P2OUT |= BIT5;              // Habilitar pullup

    P2SEL &= ~BIT1;             // Botão S1
    P2DIR &= ~BIT1;
    P2REN |= BIT1;
    P2OUT |= BIT1;
    P2IE |= BIT1;
    P2IES |= BIT1;
    do {                        // Apaga pedido de interrupção inicial
        P2IFG = 0;
    } while (P2IFG != 0);
}
/* Funçao que configura timers */
void init_timer()
{
    TA0CTL = TASSEL_1 | MC__STOP;       // Timer do ADC: ACLK, 1 Hz
    TA0CCR0 = 32768;
    TA0CCR1 = 16384;
    TA0CCTL1 = OUTMOD_6;
    TA0CTL |=(MC__UP | TACLR);

    TA1CTL = TASSEL_1 | MC__STOP | TACLR | ID_2;    // Timer para mandar mensagens (interrupção) de regar planta.
    TA1CCR0 = 32768;                                // ACLK, DIV 4 (0.25 Hz)
    TA1CCTL1 |= CCIE;        // Interrupt Enabled
}
/* Funçao que configura ADC para converter entrada P6.0 */
void init_ADC()
{
    P6SEL |= BIT0;                  // Entrada analógica do sensor de umidade
    ADC12CTL0 &= ~ADC12ENC;         //Desabilitar para configurar

    ADC12CTL0 = ADC12ON|ADC12SHT0_3;    //Ligar ADC

    ADC12CTL1 = ADC12CSTARTADD_0 |
                ADC12SHS_1 |    //Disp com TA0.1
                ADC12DIV_0 |
                ADC12SSEL_0 |
                ADC12CONSEQ_3;

    ADC12CTL2 = ADC12RES_2; //12 bits
    //Configurações dos canais (será feita a média de 4 canais para maior precisão)
    ADC12MCTL0 = ADC12SREF_0 | ADC12INCH_0;
    ADC12MCTL1 = ADC12SREF_0 | ADC12INCH_0;
    ADC12MCTL2 = ADC12SREF_0 | ADC12INCH_0;
    ADC12MCTL3 = ADC12SREF_0 | ADC12INCH_0|ADC12EOS;

    //Liga o ADC.
    ADC12CTL0 |= ADC12ENC;
    ADC12IE = ADC12IE3;
}
/* Funçao que configura comunicação com bluetooth */
void init_UART()
{
    P3SEL |= BIT3 | BIT4;               // Seleciona TX e RX
    UCA0CTL1 |= UCSWRST;
    UCA0CTL1 |= UCSSEL_1;
    UCA0BR0 = 0x03;                     // BR do MSP precisa ser igual a BR do bluetooth
    UCA0BR1 = 0x00;
    UCA0MCTL = UCBRS_3 | UCBRF_0;
    UCA0CTL1 &= ~UCSWRST;

}
/* --------- FUNÇÕES DE INTERRUPÇÃO -------------- */
/* Interrupção para fazer médias das medidas obtidas no ADC*/
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_interrupt(void)
{
    switch (_even_in_range(ADC12IV, 0x24))
    {
    case ADC12IV_NONE:
        break;
    case ADC12IV_ADC12OVIFG:
        break;
    case ADC12IV_ADC12TOVIFG:
        break;
    case ADC12IV_ADC12IFG0:
        break;
    case ADC12IV_ADC12IFG1:
        break;
    case ADC12IV_ADC12IFG2:
        break;
    case ADC12IV_ADC12IFG3:
        umidade = (ADC12MEM0+ADC12MEM1+ADC12MEM2+ADC12MEM3) >> 2;       // Faz a soma e divide por 4
        umidade -= 4095;    // 0 repesenta um ambiente umido, e 4095 um ambiente seco, por ser contraintuitivo, os valores da escala foram invertidos
        umidade *= -1;      // Ou seja, 4095 é um ambiente umido e 0 é um ambiente seco, dando sentido ao nome da variável (umidade)
        umidade_baixa = (umidade < UMIDADE_BAIXA_LIMITE) ? 1 : 0;       // Se umidade < Limite => umidade_baixa = 1 (True)
    }                                                                   // Caso contrário, umidade_baixa = 0 (False)
}
/* Interrupção do botão*/
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_INTERRUPT(void)
{
    switch (_even_in_range(P2IV,0x10))
    {
        case P2IV_NONE: break;
        case P2IV_P2IFG0: break;
        case P2IV_P2IFG1:
            s1_pressed = 1;                   // Botão selecionado (variável utilizada na main)
            break;
        case P2IV_P2IFG2: break;
        case P2IV_P2IFG3: break;
        case P2IV_P2IFG4: break;
        case P2IV_P2IFG5: break;
        case P2IV_P2IFG6: break;
        case P2IV_P2IFG7: break;
        default: break;
    }
}
/* Interrupção responsável pelo Timer que envia mensagens de "regue a planta" */
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer1_A1(void) {
    switch (TA1IV)
    {
        case TA1IV_TA1CCR1:
            for (i = 0; i < 15; i++)
            {
                UCA0TXBUF = regue[i];       // Enviar mensagem de regar planta
                while (!(UCA0IFG&UCTXIFG));
            }
            break;
    }
}
