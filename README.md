# Practica-11-BASE-DE-DATOS

Dentro de este repositorio se muestra la manera de programar una ESP32 con el sensor DHT11, el sensor Ultrasonico y además, que los datos obtenidos se vean reflejados en el programa Node-RED y se vacien a una base de datos con el progama XAMPP a través de una conexión WIFI.

## Introducción

### Descripción

La ```Esp32``` la utilizamos en un entorno de adquision de datos, lo cual en esta practica ocuparemos un sensor ```DTH11``` para adquirir temperatura y humedad del entorno, el sensor ```HC-SR04``` para el registro de distancia. Tambien utilizaremos el programa ```Node-red``` para trabajar de manera visual y esquematica la programación grafica del flujo de la información, la cual podremos observar los resultados en la interfaz de ```Node-Red``` al igual que podremos gestionar los resultados en una base de datos a traves de un servidor *WEB* con el *software* libre de ```XAMPP```; cabe aclarar que en esta practica se usara un simulador llamado [WOKWI](https://https://wokwi.com/), un programa llamado [Node-RED](http://localhost:1880/) y un *software* llamado [XAMPP](http://localhost/phpmyadmin/index.php?route=/sql&db=diplomado&table=diplomado+2024&pos=0)

## Material Necesario

Para realizar esta practica necesitas lo siguiente:
- [WOKWI](https://https://wokwi.com/)
- Programa [Node-RED](http://localhost:1880/)
- Software [XAMPP](http://localhost/phpmyadmin/index.php?route=/sql&db=diplomado&table=diplomado+2024&pos=0)
- Tarjeta ESP 32
- Sensor DHT11
- Sensor HC-SR04

## Instrucciones

### Requisitos previos

Para poder usar este repositorio necesitas entrar a la plataforma [WOKWI](https://https://wokwi.com/), tener instalado correctamente el programa [Node-RED](http://localhost:1880/) y haber instalado de manera correcta el software [XAMPP](http://localhost/phpmyadmin/index.php?route=/sql&db=diplomado&table=diplomado+2024&pos=0).

### Instrucciones de preparación del entorno en wokwi

1. Abrir la terminal de programación y colocar la siguente programación:

```
#include <ArduinoJson.h>
#include <WiFi.h>
#include <PubSubClient.h>
#define BUILTIN_LED 2
#include "DHTesp.h"
const int DHT_PIN = 15;
const int Trigger = 4;   //Pin digital 2 para el Trigger del sensor
const int Echo = 2;   //Pin digital 3 para el Echo del sensor
DHTesp dhtSensor;
// Update these with values suitable for your network.

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqtt_server = "3.65.168.153";
String username_mqtt="DanielArmenta";
String password_mqtt="12345678";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   
    // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  
    // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), username_mqtt.c_str() , password_mqtt.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(Trigger, OUTPUT); //pin como salida
  pinMode(Echo, INPUT);  //pin como entrada
  digitalWrite(Trigger, LOW);//Inicializamos el pin con 0
}

void loop() {


delay(1000);
TempAndHumidity  data = dhtSensor.getTempAndHumidity();
long t; //timepo que demora en llegar el eco
long d; //distancia en centimetros

digitalWrite(Trigger, HIGH);
delayMicroseconds(10);          //Enviamos un pulso de 10us
digitalWrite(Trigger, LOW);
  
t = pulseIn(Echo, HIGH); //obtenemos el ancho del pulso
d = t/59;             //escalamos el tiempo a una distancia en cm
  
Serial.print("Distancia: ");
Serial.print(d);      //Enviamos serialmente el valor de la distancia
Serial.print("cm");
Serial.println();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    //++value;
    //snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);

    StaticJsonDocument<128> doc;

    doc["DEVICE"] = "ESP32";
    //doc["Anho"] = 2022;
    doc["DISTANCIA"] = String(d);
    doc["TEMPERATURA"] = String(data.temperature, 1);
    doc["HUMEDAD"] = String(data.humidity, 1);
   

    String output;
    
    serializeJson(doc, output);

    Serial.print("Publish message: ");
    Serial.println(output);
    Serial.println(output.c_str());
    client.publish("CRISTEMP", output.c_str());
  }
}
```
2. Instalar la libreria de **ArduinoJson** **DTH sensor library for ESPx** **PubSubClient**. 
   - Seleccionar pestaña de Librery Manager --> Add a New library --> Colocamos el nombre de libreria
   
 ![](https://github.com/DanielX834/PRACTICA-N11/blob/main/1.jpg?raw=true)
  
3. Realizar la conexion de **DTH22** con la **ESP32** de la siguiente manera.

 ![](https://github.com/DanielX834/PRACTICA-N11/blob/main/2.jpg?raw=true)
     
  **Conexión DTH22**
  -VCC --> GND
  -SDA --> esp:15
  -NC 
  -GND  --> 5V

  **Conexión HC-SR04**
  -GND --> GND
  -ECHO --> esp: 2
  -TRIG --> esp: 4
  -VCC --> 5V

### Instrucciones de preparación del entorno en NodeRed

Para ejecutar este flow, es necesario lo siguiente
1. Arrancar el contenedor de NodeRed en **cmd** con el comando
        
        node-red

2. Dirigirse a [localhost:1880](localhost:1880)

3. En el programa de **Node-RED** primero tendran que instalar los nodos ```node-red-dashboard``` y ```node-red-node-mysql```, que se encuentran en la opcion de **Manage palette**.

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/3.jpg?raw=true)

4. En la parte izqierda de la pantalla seleccionaremos
   - 1 mqtt in
   - json
   - 4 fuction
   - 3 gauge
   - 3 chart
   - mysql

 ![](https://github.com/DanielX834/PRACTICA-N11/blob/main/3.2.jpg?raw=true)

5. Para la configuracion del mqtt in necesitaremos saber nuestra ip, que se saca de la siguiente manera:
   cmd --> nslookup broker.hivemq.com --> copiamos los numeros de la parte que dice addresses --> nos dirigimos a nodered --> seleccionamos mqtt in --> damos click en el icono del lapiz --> en la parte de server se pegara la direccion ip que copiamos --> update --> done

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/4.jpg?raw=true)

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/5.jpg?raw=true)

5. Colocar el bloque ```json``` y configurarlo el bloque con la acción de ```Always convert to JavaScript Object```  como se muestra en la imagen.

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/6.jpg?raw=true)

6. Se colocaran tres function. En el primer function se colocara el mobre de **TEMPERATURA** y posteriormente se pondra el siguiente codigo

```
msg.payload = msg.payload.TEMPERATURA;
msg.topic = "TEMPERATURA";
return msg;
```
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/7.jpg?raw=true)

7.En el segundo fuction se llamara **HUMEDAD** e igualmente se colocara un codigo

```
msg.payload = msg.payload.HUMEDAD;
msg.topic = "HUMEDAD";
return msg;
```
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/8.jpg?raw=true)

8.En el tercer fuction se llamara **Distancia** e igualmente se colocara un codigo

```
msg.payload = msg.payload.DISTANCIA;
msg.topic = "DISTANCIA";
return msg;
```
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/9.jpg?raw=true)

9. Colocamos los bloques ```Chart``` y ```Guage``` a cada una de las funciones.

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/10.jpg?raw=true)

10. Los que estan conectados a la funcion de temperatura los configuramos de la siguiente manera.

*Gauge*
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/11.jpg?raw=true)

*Chart*
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/12.jpg?raw=true)

11. Los que estan conectados a la funcion de humedad los configuramos de esta manera.

*Gauge*
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/13.jpg?raw=true)

*Chart*
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/14.jpg?raw=true)

12. Los que estan conectados a la funcion de Distancia los configuramos de esta manera.

*Gauge*
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/15.jpg?raw=true)

*Chart*
![](https://github.com/DanielX834/PRACTICA-N11/blob/main/16.jpg?raw=true)

13. El nodo de mysql le damos click --> nos vamos al lapiz --> en name y database corresponde al nombre que se genero en la plataforma phpMyAdmin --> User = root 

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/17.jpg?raw=true)

14. En la pestaña de de *Layout* crearemos otro tabulador llamado **Repositorios**, dentro de el añadiremos dos grupos uno para los indicadores y otro para las graficas; de igual manera colocaremos tres espaciadores de temperatura,humedad y distancia, los pondremos segun sea el caso y la especificación.

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/18.jpg?raw=true)

### Intrucciones de preparación del entorno phpMyAdmin  

1. En nuestra Pc buscamos **XAMPPP** en agregados recientes lo abrimos
2. Nos saldra un recuadro donde le daremos start en Apache y MySQL.
3. Damos click en Admin de MySQL
4. Nos redigirirá a una pagina que es la plataforma de **phpMyAdmin**
5. En la parte izquierda de la pantalla nos sale unas opciones en la cual seleccionaremos nuevo --> colocamos un nombre en la parte de base --> crear --> colocamos nombre de la tabla --> número de columnas son 4 en nuestro caso --> crear --> por uiltimo configuraremos las columnas

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/19.jpg?raw=true)

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/20.jpg?raw=true)

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/21.jpg?raw=true)

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/22.jpg?raw=true)

### Instrucciónes de operación

1. Iniciar simulador en [WOKWI](https://https://wokwi.com/).
2. Visualizar los datos en el monitor serial.
3. Colocar la temperatura y humedad dando *doble click* al sensor **DHT22**.
4. Colocar la distancia dando *doble click* al sensor **HC-SR04**.
5. Iniciar el simulador en [Node-RED](http://localhost:1880/) dando *click izquierdo* en el botón **Deploy** y despues abrir la interfaz dando *click izquierdo* en el boton de exportar.
6. Visualizar la interfaz.
7. Dar *click izquierdo* en la pestaña **Examinar** de la base de datos **MySQL**.
8. Visualisar los datos en la tabla creada.

## Resultados

Cuando haya funcionado, verás los valores dentro del monitor serial, la interfaz y la base de datos como se muestra en las siguentes imagenes.

Resultados en **WOKWI**

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/23.jpg?raw=true)

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/24.2.jpg?raw=true)

Resultados en **NODE-RED**

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/25.jpg?raw=true)

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/26.jpg?raw=true)

Resultados en **XAMPP**

![](https://github.com/DanielX834/PRACTICA-N11/blob/main/27.jpg?raw=true

# Créditos
Desarrollado por Ing. Daniel Armenta
