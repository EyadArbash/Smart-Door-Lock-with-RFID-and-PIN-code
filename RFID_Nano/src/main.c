#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// ---- Board: ATmega328P @ 16 MHz ----

// Pins
#define SS_PIN   PB2   // D10
#define SCK_PIN  PB5   // D13
#define MOSI_PIN PB3   // D11
#define MISO_PIN PB4   // D12

#define RST_PORT C
#define RST_BIT  0     // A0 -> PC0

// Keypad: rows -> D3,D4,D5,D6 ; cols -> D7,D8,D9

// LCD I2C address
static uint8_t LCD_ADDR = 0x3F;

// --------- LEDs ---------
// Green LED on PC1 (A1), Red LED on PC2 (A2)
static inline void leds_init(void){ DDRC |= (1<<PC1) | (1<<PC2); PORTC &= ~((1<<PC1) | (1<<PC2)); }
static inline void led_green_on(void){ PORTC |= (1<<PC1); }
static inline void led_green_off(void){ PORTC &= ~(1<<PC1); }
static inline void led_red_on(void){ PORTC |= (1<<PC2); }
static inline void led_red_off(void){ PORTC &= ~(1<<PC2); }
static inline void leds_off(void){ PORTC &= ~((1<<PC1) | (1<<PC2)); }

// --------- milllis via Timer0 ---------
volatile uint32_t millis_counter = 0;
ISR(TIMER0_COMPA_vect) { millis_counter++; }
static inline uint32_t millis(void) { uint32_t m; uint8_t sreg = SREG; cli(); m = millis_counter; SREG = sreg; return m; }

static void millis_init(void) {
  // CTC, 1ms at 16MHz with prescaler 64: OCR0A = 249
  TCCR0A = (1<<WGM01);
  TCCR0B = (1<<CS01) | (1<<CS00);
  OCR0A = 249;
  TIMSK0 = (1<<OCIE0A);
}

// --------- UART 9600 ---------
static void uart_init(void) {
  uint16_t ubrr = 103; // 16MHz, 9600
  UBRR0H = (uint8_t)(ubrr>>8);
  UBRR0L = (uint8_t)ubrr;
  UCSR0B = (1<<TXEN0);
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
  // ensure TXD (PD1) as output
  DDRD |= (1<<PD1);
}
static void uart_putc(char c) { while (!(UCSR0A & (1<<UDRE0))); UDR0 = c; }
static void uart_print(const char *s) { while (*s) uart_putc(*s++); }

// --------- SPI Master ---------
static void spi_init(void) {
  DDRB |= (1<<MOSI_PIN) | (1<<SCK_PIN) | (1<<SS_PIN);
  DDRB &= ~(1<<MISO_PIN);
  SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0); // fclk/16
  PORTB |= (1<<SS_PIN);
}
static inline void ss_low(void){ PORTB &= ~(1<<SS_PIN); }
static inline void ss_high(void){ PORTB |= (1<<SS_PIN); }
static uint8_t spi_tr(uint8_t v){ SPDR = v; while(!(SPSR & (1<<SPIF))); return SPDR; }

// --------- TWI (I2C) ---------
static void twi_init(void) { TWSR = 0; TWBR = 72; /* ~100kHz at 16MHz */ }
static uint8_t twi_wait_int(uint16_t timeout){ while(!(TWCR & (1<<TWINT))){ if(--timeout==0) return 0; } return 1; }
static uint8_t twi_start(void){ TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); return twi_wait_int(60000); }
static void twi_stop(void){ TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); }
static uint8_t twi_write(uint8_t v){ TWDR = v; TWCR = (1<<TWINT)|(1<<TWEN); return twi_wait_int(60000); }

// --------- LCD over PCF8574 ---------
static uint8_t lcd_backlight = 0x08;
static void lcd_expander(uint8_t data){ if(!twi_start()) return; if(!twi_write((LCD_ADDR<<1)|0)) { twi_stop(); return; } if(!twi_write(data | lcd_backlight)) { twi_stop(); return; } twi_stop(); }
static void lcd_pulse(uint8_t data){ lcd_expander(data | 0x04); _delay_us(2); lcd_expander(data & ~0x04); _delay_us(50); }
static void lcd_write4(uint8_t data){ lcd_expander(data); lcd_pulse(data); }
static void lcd_send(uint8_t val, uint8_t rs){ uint8_t hi = val & 0xF0; uint8_t lo = (val<<4) & 0xF0; lcd_write4(hi | (rs?0x01:0)); lcd_write4(lo | (rs?0x01:0)); }
static void lcd_cmd(uint8_t c){ lcd_send(c, 0); }
static void lcd_data(uint8_t d){ lcd_send(d, 1); }
static void lcd_clear(void){ lcd_cmd(0x01); _delay_ms(2); }
static void lcd_setCursor(uint8_t col, uint8_t row){ static const uint8_t off[4]={0x00,0x40,0x14,0x54}; lcd_cmd(0x80 | (col+off[row&0x03])); }
static void lcd_print(const char *s){ while(*s) lcd_data((uint8_t)*s++); }
static void lcd_init(void){ _delay_ms(50); lcd_write4(0x30); _delay_ms(5); lcd_write4(0x30); _delay_us(150); lcd_write4(0x30); _delay_us(150); lcd_write4(0x20); lcd_cmd(0x28); lcd_cmd(0x08); lcd_clear(); lcd_cmd(0x06); lcd_cmd(0x0C); }

static inline void lcd_backlight_on(void){
  lcd_backlight = 0x08;
  lcd_expander(0x00);
}

static inline void lcd_backlight_off(void){
  lcd_backlight = 0x00;
  lcd_expander(0x00);
}

// --------- MFRC522 minimal ---------
static inline void rst_high(void){ PORTC |= (1<<RST_BIT); }
static inline void rst_low(void){ PORTC &= ~(1<<RST_BIT); }
static void mfr_write(uint8_t reg, uint8_t val){ ss_low(); spi_tr((reg<<1)&0x7E); spi_tr(val); ss_high(); }
static uint8_t mfr_read(uint8_t reg){ ss_low(); spi_tr(((reg<<1)&0x7E)|0x80); uint8_t v = spi_tr(0x00); ss_high(); return v; }
static void mfr_setMask(uint8_t reg,uint8_t mask){ mfr_write(reg, mfr_read(reg)|mask); }
static void mfr_clrMask(uint8_t reg,uint8_t mask){ mfr_write(reg, mfr_read(reg)&(~mask)); }
static void mfr_calcCRC(uint8_t *data, uint8_t len, uint8_t *out){ 
  mfr_write(0x01,0x00); 
  mfr_write(0x0A,0x80); 
  for(uint8_t i=0;i<len;i++) 
  mfr_write(0x09,data[i]); 
  mfr_write(0x01,0x03); 
  
  for(uint16_t i=0;i<5000;i++){ 
    if(mfr_read(0x05)&0x04) 
      break; 
    } 
    out[0]=mfr_read(0x22); out[1]=mfr_read(0x21); 
  }
static uint8_t mfr_transceive(uint8_t *send, uint8_t slen, uint8_t *back, uint8_t *blen, uint8_t validBits){ 
  mfr_write(0x01,0x00); 
  mfr_write(0x02,0x77); 
  mfr_write(0x04,0x7F); 
  mfr_write(0x0A,0x80); 
  for(uint8_t i=0;i<slen;i++) 
  mfr_write(0x09,send[i]); 
  mfr_write(0x01,0x0C); 
  if(validBits) 
  mfr_write(0x0D,(validBits&7)); 
  mfr_setMask(0x0D,0x80); 
  for(uint16_t i=0;i<10000;i++){ 
    uint8_t n=mfr_read(0x04); 
    if(n&0x30) 
    break; 
  } 
  mfr_clrMask(0x0D,0x80); 
  if(mfr_read(0x06)&0x1B) 
  return 1; 
  uint8_t n=mfr_read(0x0A); 
  if(back&&blen){ 
    uint8_t l = (*blen<n)?*blen:n; 
    for(uint8_t i=0;i<l;i++) 
    back[i]=mfr_read(0x09); 
    *blen=l; } return 0; 
  }
static uint8_t mfr_REQA(uint8_t *atqa, uint8_t *alen){ 
  mfr_write(0x0D,0x07); 
  uint8_t cmd=0x26; 
  uint8_t l=*alen; 
  uint8_t r=mfr_transceive(&cmd,1,atqa,&l,7); 
  *alen=l; mfr_write(0x0D,0x00); 
  return r; 
}
static uint8_t mfr_select(uint8_t *uidOut, uint8_t *uidLen, uint8_t *sakOut){ 
  uint8_t cmd[2]={0x93,0x20}; 
  uint8_t back[10]; 
  uint8_t blen=sizeof(back); 
  if(mfr_transceive(cmd,2,back,&blen,0)!=0||blen<5) 
  return 1; 
  for(uint8_t i=0;i<4;i++) 
  uidOut[i]=back[i]; 
  *uidLen=4; uint8_t sel[9]; sel[0]=0x93; 
  sel[1]=0x70; for(uint8_t i=0;i<5;i++) 
  sel[2+i]=back[i]; 
  uint8_t crc[2]; 
  mfr_calcCRC(sel,7,crc); sel[7]=crc[0]; sel[8]=crc[1]; 
  uint8_t sakBuf[3]; 
  uint8_t sakLen=sizeof(sakBuf); 
  if(mfr_transceive(sel,9,sakBuf,&sakLen,0)!=0||sakLen<1) 
  return 1; 
  if(sakOut) 
  *sakOut=sakBuf[0]; 
  return 0; 
}
static void mfr_haltA(void){ 
  uint8_t cmd[4]={0x50,0x00,0x00,0x00}; 
  uint8_t crc[2]; 
  mfr_calcCRC(cmd,2,crc); 
  cmd[2]=crc[0]; 
  cmd[3]=crc[1]; 
  uint8_t back[2]; 
  uint8_t bl=sizeof(back); 
  (void)mfr_transceive(cmd,4,back,&bl,0); 
}
static void mfr_init(void){ 
  DDRC |= (1<<RST_BIT); 
  rst_high(); 
  spi_init(); 
  _delay_ms(50); 
  mfr_write(0x01,0x0F); 
  _delay_ms(50); 
  mfr_write(0x2A,0x80); 
  mfr_write(0x2B,0xA9); 
  mfr_write(0x2C,0x03); 
  mfr_write(0x2D,0xE8); 
  mfr_write(0x15,0x40); 
  mfr_write(0x11,0x3D); 
  uint8_t v=mfr_read(0x14); 
  if((v&0x03)!=0x03) 
  mfr_write(0x14, v|0x03); 
}

// static uint8_t mfr_version(void){ return mfr_read(0x37); }

// --------- Keypad scan ---------
static void keypad_init(void){
  // rows as outputs high
  DDRD |= (1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6);
  PORTD |= (1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6);
  // cols as inputs pullups: PD7, PB0, PB1
  DDRD &= ~(1<<PD7); PORTD |= (1<<PD7);
  DDRB &= ~((1<<PB0)|(1<<PB1)); PORTB |= (1<<PB0)|(1<<PB1);
}
static char keymap[4][3]={{'1','2','3'},{'4','5','6'},{'7','8','9'},{'*','0','#'}};
static char keypad_getKey(void){
  for(uint8_t r=0;r<4;r++){
    // set row low
    PORTD |= (1<<PD3)|(1<<PD4)|(1<<PD5)|(1<<PD6);
    PORTD &= ~(1<<(PD3+r));
    _delay_us(5);
    // read columns
    uint8_t c0 = !(PIND & (1<<PD7));
    uint8_t c1 = !(PINB & (1<<PB0));
    uint8_t c2 = !(PINB & (1<<PB1));
    if(c0||c1||c2){
      // capture column before waiting for release
      uint8_t col = c0 ? 0 : (c1 ? 1 : 2);
      _delay_ms(50); // debounce
      // wait until key released
      while(!(PIND & (1<<PD7)) || !(PINB & (1<<PB0)) || !(PINB & (1<<PB1))) { /* spin */ }
      PORTD |= (1<<(PD3+r));
      return keymap[r][col];
    }
    PORTD |= (1<<(PD3+r));
  }
  return 0;
}

// --------- App State ---------
// --------- Persistent Storage (EEPROM) ---------
#define STORAGE_SIGNATURE 0xA5
#define MAX_UIDS 8
typedef struct {
  uint8_t signature;
  char pin[4];
  uint8_t uidCount;
  uint8_t uids[MAX_UIDS][4];
} Storage;
static Storage EEMEM ee_storage;

static char currentPIN[5] = { '1','2','3','4','\0' };
static uint8_t authorizedCount = 0;
static uint8_t authorizedUIDs[MAX_UIDS][4];

// Default admin UID (current project default). Show this tag to enter admin.
static const uint8_t admin_uid[4] = {54,18,31,126};

static bool enteringPIN=false; static char enteredPIN[8]={0}; static uint8_t enteredLen=0; static uint32_t lastAction=0; static bool systemIdle=true;

// --- Energiespar-Parameter ---
// Zeit in ms bis LCD aus und Sleep, wenn keine Aktion passiert und keine Karte benutzt wird
#define LCD_SLEEP_TIMEOUT_MS 10000UL

static void storage_save(void){
  Storage s;
  s.signature = STORAGE_SIGNATURE;
  for(uint8_t i=0;i<4;i++) s.pin[i]=currentPIN[i];
  s.uidCount = authorizedCount;
  for(uint8_t i=0;i<MAX_UIDS;i++) for(uint8_t j=0;j<4;j++) s.uids[i][j] = (i<authorizedCount?authorizedUIDs[i][j]:0);
  eeprom_write_block((const void*)&s, (void*)&ee_storage, sizeof(Storage));
}

static void storage_load(void){
  Storage s; eeprom_read_block((void*)&s, (const void*)&ee_storage, sizeof(Storage));
  if(s.signature != STORAGE_SIGNATURE){
    // initialize defaults: PIN 1234, empty list
    currentPIN[0]='1'; currentPIN[1]='2'; currentPIN[2]='3'; currentPIN[3]='4'; currentPIN[4]='\0';
    authorizedCount = 0;
    storage_save();
    return;
  }
  {
    for(uint8_t i=0;i<4;i++) { currentPIN[i]=s.pin[i]; }
    currentPIN[4]='\0';
  }
  authorizedCount = (s.uidCount>MAX_UIDS)?MAX_UIDS:s.uidCount;
  for(uint8_t i=0;i<authorizedCount;i++) for(uint8_t j=0;j<4;j++) authorizedUIDs[i][j]=s.uids[i][j];
}

static bool uid_equal(const uint8_t *a, const uint8_t *b){ for(uint8_t i=0;i<4;i++) if(a[i]!=b[i]) return false; return true; }
static int16_t find_uid(const uint8_t *uid){ for(uint8_t i=0;i<authorizedCount;i++){ if(uid_equal(uid, authorizedUIDs[i])) return (int16_t)i; } return -1; }
static bool add_uid(const uint8_t *uid){ if(authorizedCount>=MAX_UIDS) return false; if(find_uid(uid)>=0) return true; for(uint8_t j=0;j<4;j++) authorizedUIDs[authorizedCount][j]=uid[j]; authorizedCount++; storage_save(); return true; }
static bool remove_uid(const uint8_t *uid){ int16_t idx=find_uid(uid); if(idx<0) return false; for(uint8_t i=idx;i+1<authorizedCount;i++) for(uint8_t j=0;j<4;j++) authorizedUIDs[i][j]=authorizedUIDs[i+1][j]; authorizedCount--; storage_save(); return true; }

static bool adminMode=false; static uint8_t adminState=0; // 0=menu,1=add,2=del,3=chgPIN
static char newPIN[8]={0}; static uint8_t newPINLen=0;

static void show_admin_menu(void){
  lcd_clear();
  lcd_setCursor(0,0); lcd_print("1+Tag 2-Del 3-PIN");
  lcd_setCursor(0,1); lcd_print("4-Exit");
}

static void reset_state(void){ enteringPIN=false; enteredLen=0; enteredPIN[0]='\0'; if(!adminMode){ lcd_clear(); lcd_setCursor(0,0); lcd_print("Show your card :)"); } systemIdle=true; lastAction=millis(); leds_off(); }

// Watchdog-Interrupt (wird nur für periodisches Aufwecken benutzt)
ISR(WDT_vect){
  // nichts zu tun, Aufwachen reicht
}

// ca. 250 ms Wake-Intervall
static void wdt_enable_interrupt_250ms(void){
  cli();
  // Konfigurationsänderung erlauben
  WDTCSR = (1<<WDCE) | (1<<WDE);
  // Nur Interrupt, kein Reset; ca. 250 ms: WDP2|WDP1 (0b110)
  WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1);
  sei();
}

static void sleep_if_idle(void){
  // Nur reagieren, wenn System im Idle ist, kein PIN-Eintrag und kein Admin-Menü
  if(!systemIdle || enteringPIN || adminMode) return;

  uint32_t now = millis();
  if((now - lastAction) < LCD_SLEEP_TIMEOUT_MS) return;

  // LCD-Hintergrundlicht und LEDs aus
  lcd_backlight_off();
  leds_off();

  // Energiesparen: wiederholt schlafen, kurz aufwachen per WDT, nach Karte/Taste schauen
  while(systemIdle && !enteringPIN && !adminMode){
    wdt_enable_interrupt_250ms();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sei();
    sleep_cpu();    // schläft bis WDT-Interrupt (oder INT0) aufweckt
    sleep_disable();
    wdt_disable();  // sicherheitshalber aus

    // Nach dem Aufwachen: kurz nach Karte schauen (nur bei bestätigter UID Backlight an)
    uint8_t atqa[2]; uint8_t alen=2;
    if(mfr_REQA(atqa,&alen)==0){
      uint8_t uid[10]; uint8_t uidLen=0; uint8_t sak=0;
      if(mfr_select(uid,&uidLen,&sak)==0 && uidLen>=4){
        lcd_backlight_on();
        systemIdle=false; // wir haben eine Karte, normaler Loop verarbeitet weiter
        lastAction = millis();
        break;
      }
    }

    // Keine Aktivität -> erneut schlafen
  }
}

static void handlePINEntry(void){
  char key = keypad_getKey(); if(!key) return;
  if(key=='#'){
    enteringPIN=false;
    if(strcmp(enteredPIN, currentPIN)==0){ lcd_clear(); lcd_print("Access granted"); led_green_on(); led_red_off(); }
    else { lcd_clear(); lcd_print("Wrong PIN"); led_red_on(); led_green_off(); }
    lastAction=millis();
  } else if(key=='*'){
    if(enteredLen){ enteredLen--; enteredPIN[enteredLen]='\0'; lcd_setCursor(enteredLen,1); lcd_print(" "); lcd_setCursor(enteredLen,1); }
  } else if(key>='0'&&key<='9'){
    if(enteredLen<4){ enteredPIN[enteredLen++]=key; enteredPIN[enteredLen]='\0'; lcd_setCursor(enteredLen-1,1); lcd_print("*"); }
  }
}

int main(void){
  cli();
  millis_init();
  uart_init();
  
  twi_init();
  
  lcd_init();
  // Start: Hintergrundbeleuchtung an
  lcd_backlight_on();
  lcd_setCursor(0,0); lcd_print("Show your card :)");
  keypad_init();
  spi_init();
  mfr_init();
  leds_init();
  leds_off();
  // Power-on self-test: blink green then red
  led_green_on(); _delay_ms(300); leds_off(); _delay_ms(150);
  led_red_on(); _delay_ms(300); leds_off();
  storage_load();
  sei();

  
  lastAction = millis();
  

  while(1){
    // Timeout reset
    if((millis()-lastAction>5000) && !enteringPIN && !systemIdle){ reset_state(); }

    // RFID prüfen
    if(!enteringPIN){
      uint8_t atqa[2]; uint8_t alen=2; if(mfr_REQA(atqa,&alen)==0){
        uint8_t uid[10]; uint8_t uidLen=0; uint8_t sak=0;
        if(mfr_select(uid,&uidLen,&sak)==0){
          systemIdle=false;
          // Admin flow handling (wait for card during add/del)
          if(adminMode){
            if(uidLen>=4){
              if(adminState==1){ // add
                if(add_uid(uid)) { lcd_clear(); lcd_print("Added card"); led_green_on(); _delay_ms(300); }
                else { lcd_clear(); lcd_print("Add failed"); led_red_on(); _delay_ms(300); }
                leds_off(); show_admin_menu(); adminState=0; lastAction=millis();
              } else if(adminState==2){ // delete
                if(remove_uid(uid)) { lcd_clear(); lcd_print("Deleted card"); led_green_on(); _delay_ms(300); }
                else { lcd_clear(); lcd_print("Not found"); led_red_on(); _delay_ms(300); }
                leds_off(); show_admin_menu(); adminState=0; lastAction=millis();
              }
            }
          } else {
            // Check admin first
            bool isAdmin = (uidLen>=4) && uid_equal(uid, admin_uid);
            if(isAdmin){
              lcd_backlight_on();
              adminMode=true; adminState=0; show_admin_menu(); lastAction=millis(); leds_off();
            } else {
              // Check authorized list
              bool authorized = (uidLen>=4) && (find_uid(uid)>=0);
              if(authorized){
                lcd_backlight_on();
                lcd_clear(); lcd_print("Enter PIN:"); uart_print("Correct card, enter PIN...\r\n");
                enteringPIN=true; enteredLen=0; enteredPIN[0]='\0'; lcd_setCursor(0,1); leds_off();
              }
              else {
                lcd_backlight_on();
                lcd_clear(); lcd_print("Access denied"); uart_print("Unknown card!\r\n"); led_red_on(); led_green_off(); lastAction=millis();
              }
            }
          }
          mfr_haltA();
        }
      }
    }

    if(enteringPIN) handlePINEntry();

    // Admin keypad handling
    if(adminMode && !enteringPIN){
      char key = keypad_getKey(); if(key){
        if(adminState==0){
          if(key=='1'){ adminState=1; lcd_backlight_on(); lcd_clear(); lcd_print("Add: show card"); }
          else if(key=='2'){ adminState=2; lcd_backlight_on(); lcd_clear(); lcd_print("Del: show card"); }
          else if(key=='3'){ adminState=3; newPINLen=0; newPIN[0]='\0'; lcd_backlight_on(); lcd_clear(); lcd_print("New PIN:"); lcd_setCursor(0,1); }
          else if(key=='4'){ adminMode=false; adminState=0; reset_state(); }
        } else if(adminState==3){
          if(key=='#'){
            if(newPINLen==4){ for(uint8_t i=0;i<4;i++) currentPIN[i]=newPIN[i]; currentPIN[4]='\0'; storage_save(); lcd_clear(); lcd_print("PIN updated"); led_green_on(); _delay_ms(300); leds_off(); show_admin_menu(); adminState=0; }
            else { lcd_clear(); lcd_print("PIN 4 digits"); _delay_ms(500); lcd_backlight_on(); lcd_clear(); lcd_print("New PIN:"); lcd_setCursor(0,1); newPINLen=0; newPIN[0]='\0'; }
          } else if(key=='*'){
            show_admin_menu(); adminState=0;
          } else if(key>='0' && key<='9'){
            if(newPINLen<4){ newPIN[newPINLen++]=key; newPIN[newPINLen]='\0'; lcd_print("*"); }
          }
        }
        lastAction=millis();
      }
    }

    // Energiesparen: nach längerer Inaktivität LCD aus und ggf. Sleep
    sleep_if_idle();
  }
}


