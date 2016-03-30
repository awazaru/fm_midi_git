/* Name: Frequency modulation oscillator with MIDI
 * Author:awazaru
 * Micro Controller :Atmega328P
 * Date :10/03/2016
 */

#include <avr/io.h>
#include <avr/interrupt.h>

/*シリアル通信UART*/
#define FOSC   20000000//動作周波数
#define BAUD   9600//ボーレート
#define MYUBRR (FOSC/16/BAUD)-1
/*アスキーコード*/
#define LF tx_usart(0x0a)//改行
#define CR tx_usart(0x0d)//復帰
#define HT tx_usart(0x09)//水平タブ

/*サンプリング周波数*/

#define TABLE 200//sinテーブルの要素数
#define SAMPLE_F 20000//サンプリングF
#define OCR1A_F (FOSC/SAMPLE_F)-1
#define BASIC_F (SAMPLE_F/TABLE)//基本周波数 サンプリング周波数/c_sinの要素数
#define FREQ    440//出したい周波数
#define CNT_CAL (FREQ/BASIC_F)

/*sinテーブル*/
/*キャリア用テーブル(変調をかけられる側)*/
uint8_t c_sin[TABLE]={131,135,139,144,148,152,156,159,163,167,171,175,179,182,185,189,193,196,199,203,205,209,212,214,218,221,223,226,228,231,233,235,237,240,241,242,245,246,247,249,250,251,252,252,254,254,255,255,255,255,255,255,255,254,254,252,252,251,250,249,247,246,245,242,241,240,237,235,233,231,228,226,223,221,218,214,212,209,205,203,199,196,193,189,185,182,179,175,171,167,163,159,156,152,148,144,139,135,131,128,124,120,116,111,107,103,99,96,92,88,84,80,77,73,70,66,62,59,56,52,50,46,43,41,37,34,32,29,27,24,22,20,18,15,14,13,10,9,8,6,5,4,3,3,1,1,0,0,0,0,0,0,0,1,1,3,3,4,5,6,8,9,10,13,14,15,18,20,22,24,27,29,32,34,37,41,43,46,50,52,56,59,62,66,70,73,77,80,84,88,92,96,99,103,107,111,116,120,124,128};
/*モジュレーション用テーブル(変調をかける側)*/
int8_t m_sin[TABLE]={0,1,1,1,2,2,2,2,3,3,3,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,8,8,8,8,8,8,9,9,9,9,9,9,9,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,9,9,9,9,9,9,9,8,8,8,8,8,8,7,7,7,7,6,6,6,6,5,5,5,5,4,4,4,3,3,3,2,2,2,2,1,1,1,0,0,0,-1,-1,-1,-2,-2,-2,-2,-3,-3,-3,-4,-4,-4,-5,-5,-5,-5,-6,-6,-6,-6,-7,-7,-7,-7,-8,-8,-8,-8,-8,-8,-9,-9,-9,-9,-9,-9,-9,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-10,-9,-9,-9,-9,-9,-9,-9,-8,-8,-8,-8,-8,-8,-7,-7,-7,-7,-6,-6,-6,-6,-5,-5,-5,-5,-4,-4,-4,-3,-3,-3,-2,-2,-2,-2,-1,-1,-1,0,0};

/*カウンタ変数*/
float sin_count=0;
uint16_t mod_count =0;
float cnt_speed = CNT_CAL;
uint8_t cnt_serial=0;//シリアル通信割り込み処理内で使うカウンタ

/*グロ-バル変数*/
uint16_t cnt_point = 0;//sinテーブルのカウンタの更新周期を決めるカウンタ ISR(TIMER1_COMPA_vect)内で使用
uint16_t buf_f=0;//シリアル通信割り込み処理で受け取った周波数情報を記録する変数
uint8_t array_buf_f[10]={"0"};//割り込みで受けとった数字一桁ずつを保存していく配列

void serial_ini(void){
  UBRR0=MYUBRR;
  UCSR0A=0b00000000;//受信すると10000000 送信有効になると00100000
  UCSR0B=0b10011000;//送受信有効,受信割り込み許可
  UCSR0C=0b00000110;//データ8bit、非同期、バリティなし、stop1bit
}

unsigned char rx_usart(void)//受信用関数
{
  while( !(UCSR0A & (1<<RXC0)) );                //受信完了待機
  return UDR0;                                //受信ﾃﾞｰﾀ返す
}

void tx_usart(unsigned char data)//送信用関数
{
  while( !(UCSR0A & (1<<UDRE0)) );        //送信ﾊﾞｯﾌｧ空き待機
  UDR0 = data;
}

void puts_tx(char *str)//文字列送信用関数
{
  while( *str != 0x00 ) //nullになるまで実行
    {
	 tx_usart(*str);
	 str++;                                    //ｱﾄﾞﾚｽ+10    }
    }
}

void tx_line_number(unsigned int val){
  unsigned char a4,a3,a2,a1;//各桁
  a4=(val/1000)+0x30;
  a3=(val/100)%10+0x30;
  a2=(val/10)%10+0x30;
  a1=(val%10)+0x30;

  tx_usart(a4);
  tx_usart(a3);
  tx_usart(a2);
  tx_usart(a1);
  return;
}

ISR(USART_RX_vect){//シリアル通信割り込み
  PORTB|=_BV(PIN2);
   unsigned char ch=0;
   ch = rx_usart();
   /*
   if(ch==0x31){
	tx_usart(ch);
	PORTB|=_BV(PIN2);
   }else if(ch==0x32){
	tx_usart(ch);
	PORTB&=~_BV(PIN2);
	}
   */
   puts_tx("PLEASE INPUT NUMBER(0000~9999)");
   uint8_t i=0;
   HT;
   for(i=1;i<=4;i++){//入力し直しの場合のためにあえてi=1からスタートする
	ch = rx_usart();
	if((ch-0x30)>=0&&(ch-0x30)<=9){//0~9の間の値のみ受け付ける
	tx_usart(ch);
	array_buf_f[4-i]=ch-0x30;
	}else {
	  LF;
	  puts_tx("ERROR PLEASE INPUT NUMBER");
	  LF;
	  i=0;//リセット 入力しなおし
	}
   }
   buf_f=0;//初期化
   for(i=1;i<=4;i++){//上のfor文と合わせるためにi=1からスタート
	buf_f=10*buf_f+array_buf_f[4-i];
   }
   HT;//水平タブ
   puts_tx("OK");
   LF;//改行
   puts_tx("Change to ");
   tx_line_number(buf_f);
   puts_tx("Hz");
   LF;
   puts_tx("PLEASE PUSH ANY KEY");
   LF;
   cnt_speed=(buf_f/BASIC_F);
   PORTB&=~_BV(PIN2);
}


void timer_ini(void){
  //16bitタイマ タイマ割り込み用
  TCCR1B|=_BV(WGM12)|_BV(CS10);//0CR1A基準 比較一致タイマ 分周なし
  TIMSK1|=_BV(OCIE1A);//比較A一致割り込み有効
  OCR1A=OCR1A_F;//サンプリング周波数
  
  //8bitタイマ PWM用
  TCCR0A|=_BV(COM0A1)|_BV(WGM01)|_BV(WGM00);//高速PWM 比較一致でLOW OC0Aピン
  TCCR0B|=_BV(CS00);
  OCR0A=0;
}

ISR(TIMER1_COMPA_vect){//タイマ1A比較一致割り込み
  mod_count =sin_count+m_sin[mod_count];
  OCR0A=c_sin[mod_count];
  sin_count = sin_count+cnt_speed;//マイコンで除算をさせるのは負荷が大きい
  if(sin_count>=TABLE){
    sin_count=0;
  }
}

void pin_ini(void){
  DDRD|=_BV(PIN6);
  DDRB|=_BV(PIN2);
}

int main(void)
{
  pin_ini();
  timer_ini();
  serial_ini();
  sei();//割り込み有効
    for(;;){
    }
    return 0;
}
