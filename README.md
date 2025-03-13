# Autonomous-Mobile-Robot
Desenvolvimento de um robô móvel inteligente com foco em otimização por meio de Inteligência Artificial.

---

## Configuração de Credenciais  

O projeto utiliza um arquivo chamado `env.h` para armazenar informações sensíveis, como credenciais de Wi-Fi e configurações de rede. Esse arquivo **não está incluído no repositório** (listado no `.gitignore`) para proteger dados privados.  

### Criando o arquivo `env.h`  

Antes de compilar o projeto, crie um arquivo chamado `env.h` na pasta do código-fonte e adicione o seguinte conteúdo, substituindo os valores conforme necessário:  

```cpp
#ifndef ENV_H
#define ENV_H

// Credenciais Wi-Fi
#define WIFI_SSID "SEU_WIFI"
#define WIFI_PASSWORD "SUA_SENHA"

// Configurações adicionais
#define COPPELIA_IP "SEU_IP"
#define COPPELIA_PORT 19997

#endif // ENV_H
```  

### Uso no Código  

O arquivo `env.h` é incluído no código principal do projeto para acessar essas informações sem expô-las publicamente.  

---
