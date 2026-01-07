# Projeto de Lançamento de Balão

Este repositório contém os códigos utilizados para o lançamento de um balão meteorológico. O balão foi equipado com um módulo LoRa para comunicação com uma estação terrestre. Além disso, foi utilizado um código para um planador chamado ASA, com o objetivo de ajustar sua posição para um voo planado após o lançamento.

## Visão Geral

O projeto envolve a comunicação via LoRa entre o balão e uma estação terrestre, permitindo o envio e recebimento de dados durante o voo. O planador ASA foi integrado para otimizar a trajetória de descida do balão, garantindo um voo planado controlado.

## Estrutura do Projeto

### BEC_BRISA_DISTANCE_TEST_RECEIVER_V2
Contém o código para o receptor LoRa na estação terrestre. O arquivo principal é `BEC_BRISA_DISTANCE_TEST_RECEIVER_V2.ino`, que deve ser carregado no microcontrolador da estação terrestre.

### BEC_BRISA_DISTANCE_TEST_TRANSMITTER_V2
Contém o código para o transmissor LoRa no balão. O arquivo principal é `BEC_BRISA_DISTANCE_TEST_TRANSMITTER_V2.ino`, que deve ser carregado no microcontrolador do balão.

### BRISA_ASA
Contém o código para o planador ASA. O arquivo principal é `BRISA_ASA.ino`, que deve ser carregado no microcontrolador do planador.

### libraries
Contém diversas bibliotecas utilizadas no projeto, incluindo:
- `Adafruit_BMP085_Unified`: Biblioteca para sensor de pressão BMP085.
- `Adafruit_BusIO`: Biblioteca de abstração de barramento I2C e SPI.
- `Adafruit_GFX_Library`: Biblioteca gráfica para displays.
- `Adafruit_MPU6050`: Biblioteca para o sensor de movimento MPU6050.
- `Adafruit_SSD1306`: Biblioteca para display OLED SSD1306.
- `DHT_kxn`: Biblioteca para sensor de umidade e temperatura DHT.

## Diretrizes de Contribuição

Para contribuir com este projeto, siga as diretrizes abaixo:

1. **Fork o Repositório:**
   - Faça um fork deste repositório para sua conta no GitHub.

2. **Clone o Fork:**
   - Clone o fork para sua máquina local usando `git clone https://github.com/seu-usuario/seu-fork.git`.

3. **Crie uma Branch:**
   - Crie uma branch para sua contribuição usando `git checkout -b minha-contribuicao`.

4. **Faça suas Alterações:**
   - Adicione ou modifique o código conforme necessário.

5. **Commit suas Alterações:**
   - Faça commit das suas alterações com mensagens claras usando `git commit -m "Minha contribuição"`.

6. **Push para o Fork:**
   - Faça push das suas alterações para o seu fork no GitHub usando `git push origin minha-contribuicao`.

7. **Abra um Pull Request:**
   - Abra um pull request para a branch principal deste repositório.

8. **Aguarde a Revisão:**
   - Aguarde a revisão e possíveis solicitações de alterações.

## Contato

Para dúvidas ou sugestões, entre pelos contatos disponíveis em 'https://www.riomakerspace.com.br/sobre-o-rms/fale-conosco'

## Licença

Este projeto está sob a licença MIT. Veja o arquivo LICENSE para mais detalhes.
