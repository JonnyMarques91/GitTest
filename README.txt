Descrição do Projeto:

Sensor de umidade: Utilizar para monitorar três diferentes níveis de umidade do solo: Seco, úmido ou encharcado. A bomba será atuada de acordo com a classificação do estado do solo;

Bomba de Água + Ponte H Será controlada por PWM e um determinado tempo de atuação (Timer). A bomba será acionada (ou não) após verificação da estrutura condicional (Estado crítico?);

Bluetooth: Enviará periodicamente (ou mediante detecção de presença ou alta temperatura) as informações de todos os sensores contidos na planta, como umidade, luminosidade, temperatura, estado da bomba, presença de alguém próximo a planta;

Sensor de presença: Caso alguém seja detectado nas proximidades da planta, imediatamente todas as informações de sensoriamento serão coletadas e enviadas via Bluetooth.

Sensor de temperatura (Modificado): Caso a temperatura seja muito alta e o solo não esteja encharcado, a bomba atuará brevemente de forma a “refrescar” a planta.
Modificação: Não foi possível utilizar o sensor de Umidade+Temperatura BME280, por falta de conhecimento em SPI/I2C. Assim, foi utlizado o sensor de temperatura do próprio SAM4SD32C. 

Módulos Utilizados:

Sensor de Temperatura: BME280 Sensor de Umidade de solo: FC-28 * Sensor de Iluminação: LDR Sensor de Presença: PIR BISS0001 Bluetooth: JDY09 AT09 * Bomba de Água: Submersible Pump 12VDC series * Ponte H: L298 *