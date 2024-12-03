// include the library
#include <RadioLib.h>
// include the hardware abstraction layer
#include "hal/RPi/PiHal.h"

#define FREQUENCY 433.0
#define BANDWIDTH 500.0
#define SPREADING_FACTOR 7
#define CODING_RATE 5

// create a new instance of the HAL class
PiHal* hal = new PiHal(0);    // 0=>SPI0  1=>SPI1

// pinout corresponds to the Waveshare LoRaWAN Hat
#define NSSpin    25
#define DIO0pin   27
#define DIO1pin   24
#define NRSTpin   22
#define BUSYpin   RADIOLIB_NC   // BUSYpin not connected
#define MAX_PACKET_LENGHT  64
// now we can create the radio module
SX1278 radio = new Module(hal, NSSpin, DIO0pin, NRSTpin, DIO1pin);

// Flag per la ricezione e timeout
volatile bool detectedFlag  = false;
volatile bool timeoutFlag = false;
// flag to indicate if we are currently receiving
bool receiving = false;
// Callback per pacchetto ricevuto
void onReceive() { detectedFlag = true; }
// Callback per timeout durante la scansione del canale
void onTimeout() { timeoutFlag = true; }

// Control return state of radio.begin function
void checkInitializationState(const int& initState){
  // Case of most frequent errors-state
  switch (initState) {
    case RADIOLIB_ERR_NONE:
      printf("Inizializzato con successo!\n"); break;
    case RADIOLIB_ERR_INVALID_GAIN:
      printf("Errore: Gain non valido.\n"); break;
    case RADIOLIB_ERR_INVALID_FREQUENCY:
      printf("Errore: Frequenza non valida.\n"); break;
    case RADIOLIB_ERR_CHIP_NOT_FOUND:
      printf("Errore: Modulo LoRa non trovato.\n"); break;
    case RADIOLIB_ERR_PACKET_TOO_LONG:
      printf("Errore: Pacchetto troppo lungo.\n"); break;      
    default:
      printf("Errore sconosciuto. Codice: %d\n", initState); break;
  }
  // exit in case of error(s)
  if (initState != RADIOLIB_ERR_NONE) { exit(EXIT_FAILURE); }
  return;
}
// Control return state of radio.startChannelScan function
void checkChannelScanState(const int& scanState, bool restart=false){
  if (scanState == RADIOLIB_ERR_NONE) {
    printf("success!\n");
  } else {
    if(restart){
      printf("Errore nel riavvio della scansione, codice: %d\n", scanState);
    } else {
      printf("failed, code %d\n", scanState);
    }
  }
}
// Control return state of radio.readData function
void checkReadState(const int& readState, uint8_t (&str)[MAX_PACKET_LENGHT]){
  if (readState == RADIOLIB_ERR_NONE) {
          // packet was successfully received
          printf("[SX1278] Received packet!\n");
          // print data of the packet
          printf("[SX1278] Data:\t\t%s\n", str);
          // print RSSI (Received Signal Strength Indicator)
          printf("[SX1278] RSSI:\t\t%f dBm\n", radio.getRSSI());
          // print SNR (Signal-to-Noise Ratio)
          printf("[SX1278] SNR:\t\t%f dB\n", radio.getSNR());
          // print frequency error
          printf("[SX1278] Frequency error:\t%f Hz\n", radio.getFrequencyError());
        } else if (readState == RADIOLIB_ERR_CRC_MISMATCH) {
          // packet was received, but is malformed
          printf("[SX1278] CRC error!\n");
        } else {
          // some other error occurred
          printf("Failed, code %d\n", readState);
        }
}
// Control return state of radio.startReceive function
void checkReceivingState(const int& receivingState){
  if(receivingState == RADIOLIB_ERR_NONE){
    printf("success!\n");
  }
  else{
    printf("failed, code: %d\n", receivingState);
  }
}

// the entry point for the program
int main(int argc, char** argv) {
  // initialize just like with Arduino
  printf("Initializing... ");
  int initState = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE);
  checkInitializationState(initState);

  // Imposta le callback per la ricezione e il timeout
  radio.setDio1Action(onReceive, PI_RISING);
  radio.setDio0Action(onTimeout, PI_RISING);
  // set the function that will be called when new packet is received
  radio.setPacketReceivedAction(onReceive);

  // start scanning the channel
  printf("Starting scan for LoRa preamble... ");
  int scanState = radio.startChannelScan();
  checkChannelScanState(scanState);

  radio.startReceive(100/*, RADIOLIB_IRQ_RX_DEFAULT_FLAGS, RADIOLIB_IRQ_RX_DEFAULT_MASK, 64*/);
  // loop forever
  while(true) {

    if(detectedFlag) {
      // check ongoing reception
      if(receiving) {
        // DIO triggered while reception is ongoing, that means we got a packet
        // reset flag first
        detectedFlag = false;
        uint8_t message[MAX_PACKET_LENGHT];
        //size_t packetLenght = radio.getPacketLength();
        printf("before readstate\n");
        int readState = radio.readData(message, MAX_PACKET_LENGHT);
        printf("after readstate\n");
        checkReadState(readState, message);
        // reception is done now
        receiving = false;
      }
      if(detectedFlag) {
        // LoRa preamble was detected, start reception with timeout of 100 LoRa symbols
        printf("Preamble detected, starting reception... ");
        int receivingState = radio.startReceive(100);
        checkReceivingState(receivingState);
        // set the flag for ongoing reception
        receiving = true;
      }

      // if we're not receiving, start scanning again
      if(!receiving) {
        // Riavvia la scansione
        scanState = radio.startChannelScan();
        checkChannelScanState(scanState, true);
      }
      // reset flags
      detectedFlag = false;
      
    } // endif
    
  }	// endwhile
  return(0);
}	// endmain