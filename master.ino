// I got the chess engine from https://home.hccnet.nl/h.g.muller/umax4_8.c
// Chess Engine Globals
// **********************************************************************
#define W while
#define M 0x88
#define S 128
#define I 8000
#define MYRAND_MAX 65535     /* 16bit pseudo random generator */
long  N, T;                  /* N=evaluated positions+S, T=recursion limit */
short Q, O, K, R, k=16;      /* k=moving side */
char *p, c[5], Z;            /* p=pointer to c, c=user input, computer output, Z=recursion counter */
char L,
w[]={0,2,2,7,-1,8,12,23},                             /* relative piece values    */
o[]={-16,-15,-17,0,1,16,0,1,16,15,17,0,14,18,31,33,0, /* step-vector lists */
     7,-1,11,6,8,3,6,                                 /* 1st dir. in o[] per piece*/
     6,3,5,7,4,5,3,6};                                /* initial piece setup      */
/* board is left part, center-pts table is right part, and dummy */  
char b[]={     
  22, 19, 21, 23, 20, 21, 19, 22, 28, 21, 16, 13, 12, 13, 16, 21,
  18, 18, 18, 18, 18, 18, 18, 18, 22, 15, 10,  7,  6,  7, 10, 15,
   0,  0,  0,  0,  0,  0,  0,  0, 18, 11,  6,  3,  2,  3,  6, 11,
   0,  0,  0,  0,  0,  0,  0,  0, 16,  9,  4,  1,  0,  1,  4,  9,
   0,  0,  0,  0,  0,  0,  0,  0, 16,  9,  4,  1,  0,  1,  4,  9,
   0,  0,  0,  0,  0,  0,  0,  0, 18, 11,  6,  3,  2,  3,  6, 11,
   9,  9,  9,  9,  9,  9,  9,  9, 22, 15, 10,  7,  6,  7, 10, 15,
  14, 11, 13, 15, 12, 13, 11, 14, 28, 21, 16, 13, 12, 13, 16, 21, 0
};
char bk[16*8+1];
unsigned int seed=0;
uint32_t  byteBoard[8];
char sym[17] = {".?pnkbrq?P?NKBRQ"};
int mn=1;
char lastH[5], lastM[5];
unsigned short ledv=1;
// **********************************************************************

// Movement Globals
// **********************************************************************
#include <AccelStepper.h>
#include <Servo.h>
AccelStepper xstepper(AccelStepper::DRIVER, 7, 8);
AccelStepper ystepper(AccelStepper::DRIVER, 6, 5);
Servo servo;
#define X_ENABLE 4
#define Y_ENABLE 3

const int xstop = 43;
const int ystop = 32;
// **********************************************************************

// Square Detection Globals
// **********************************************************************
const byte latchPin = 9;        // to latch the inputs into the registers
const byte clockPin = 13;       // I choose the SCK pin
const byte dataPin = 12;        // I choose the MISO pin
uint64_t oldOptionSwitch = 0;    // previous state of all the inputs
const int pulseWidth = 10;      // pulse width in microseconds
const char* squares[64] = {
    "e7", "e8", "d5", "d6", "a2", "a1", "a8", "a6",
    "d7", "c1", "c3", "c6", "c5", "c4", "c2", "d8",
    "e2", "e4", "e6", "d2", "d1", "d5", "e3", "e1",
    "g2", "g4", "g6", "g8", "g7", "g5", "g3", "g1",
    "h2", "h4", "h6", "h8", "h7", "h5", "h3", "h1",
    "f2", "f4", "f6", "f8", "f7", "f5", "f3", "f1",
    "d4", "b8", "a4", "a5", "a7", "a3", "b7", "d3",
    "c8", "b1", "b4", "b6", "b5", "b3", "b2", "c7"
};
// **********************************************************************

// Functions 
void home(){
  Serial.print("Starting home\n");
  xstepper.setAcceleration(300);
  ystepper.setAcceleration(300);
  xstepper.setMaxSpeed(300.0);
  ystepper.setMaxSpeed(300.0);
  while ((digitalRead(xstop) != LOW) || (digitalRead(ystop) != LOW)){
    while (digitalRead(xstop) == HIGH){
      xstepper.move(300);
      xstepper.run();
      if (digitalRead(xstop) == HIGH){
        xstepper.stop();
      }
    }
    while (digitalRead(ystop) == HIGH) {
      ystepper.move(-300);
      ystepper.run();
      if (digitalRead(ystop) == HIGH){
        ystepper.stop();
      }
    }
  }
  xstepper.setCurrentPosition(0);
  ystepper.setCurrentPosition(0);
  servo.write(0);
  Serial.print("HOME\n");
}

void movePeice(const String& peiceOne, const String& peiceTwo) {
  Serial.print("Moving piece\n");
  String startLetter = peiceOne.substring(0,1);
  String startNumber = peiceOne.substring(1);
  String finalLetter = peiceTwo.substring(0,1);
  String finalNumber = peiceTwo.substring(1);


  const int halfx = -112.5;
  const int halfy = 114;
 
  //convert letter and number into steps
  int startX = 0;
  int finalX = 0;
  switch (startLetter.charAt(0)) {
      case 'a': startX = 0; break;
      case 'b': startX = -225; break;
      case 'c': startX = -450; break;
      case 'd': startX = -675; break;
      case 'e': startX = -900; break;
      case 'f': startX = -1125; break;
      case 'g': startX = -1350; break;
      case 'h': startX = -1575; break;
  }
  switch (finalLetter.charAt(0)) {
      case 'a': finalX = 0; break;
      case 'b': finalX = -225; break;
      case 'c': finalX = -450; break;
      case 'd': finalX = -675; break;
      case 'e': finalX = -900; break;
      case 'f': finalX = -1125; break;
      case 'g': finalX = -1350; break;
      case 'h': finalX = -1575; break;
  }
  int startY = 0;
  int finalY = 0;
  switch (startNumber.charAt(0)) {
      case '1': startY = 1600; break;
      case '2': startY = 1368; break;
      case '3': startY = 1140; break;
      case '4': startY = 912; break;
      case '5': startY = 684; break;
      case '6': startY = 456; break;
      case '7': startY = 228; break;
      case '8': startY = 0; break;
  }
  switch (finalNumber.charAt(0)) {
      case '1': finalY = 1600; break;
      case '2': finalY = 1368; break;
      case '3': finalY = 1140; break;
      case '4': finalY = 912; break;
      case '5': finalY = 684; break;
      case '6': finalY = 456; break;
      case '7': finalY = 228; break;
      case '8': finalY = 0; break;
  }

  xstepper.runToNewPosition(startX);
  ystepper.runToNewPosition(startY);
  servo.write(28);
  // if the starting position is not on H
  if (xstepper.currentPosition() > -1574) {

    //move it to the right half a square
    xstepper.runToNewPosition(startX + halfx);

    // if the starting pos is not on 8
    if (ystepper.currentPosition() > 0){

      // if the ending pos is on 8
      if ( finalY == 0 ){
        
        ystepper.runToNewPosition(finalY + halfy); // move between 7 & 8
        xstepper.runToNewPosition(finalX); // go to final X
        ystepper.runToNewPosition(finalY); // move to final Y
        servo.write(0);

        // move up half square to let switch be usable
        xstepper.runToNewPosition(finalY + halfy);

      // if the final pos is not on 8
      }else{
        ystepper.runToNewPosition(finalY - halfy); // move up half square distance above target
        xstepper.runToNewPosition(finalX); // go to final X
        ystepper.runToNewPosition(finalY); // go to final Y
        servo.write(0);
        ystepper.runToNewPosition(finalY - halfy);
      }

    // if starting pos is on 8
    }else{
      
      // if ending pos is on 8
      if ( finalY == 0 ){
        ystepper.runToNewPosition(finalY + halfy); 
        xstepper.runToNewPosition(finalX);
        ystepper.runToNewPosition(finalY);
        servo.write(0);
        ystepper.runToNewPosition(finalY + halfy);

      // if ending pos is not on 8
      }else{
        ystepper.runToNewPosition(finalY - halfy); 
        xstepper.runToNewPosition(finalX);
        ystepper.runToNewPosition(finalY);
        servo.write(0);
        ystepper.runToNewPosition(finalY - halfy);
      }
    }

  // if starting pos is on H
  }else{
    
    xstepper.runToNewPosition(startX - halfx);

    // if the starting pos is not on 8
    if (ystepper.currentPosition() > 0){

      // if the ending pos is on 8
      if ( finalY == 0 ){
        
        ystepper.runToNewPosition(finalY + halfy); // move between 7 & 8
        xstepper.runToNewPosition(finalX); // go to final X
        ystepper.runToNewPosition(finalY); // move to final Y
        servo.write(0);
        ystepper.runToNewPosition(finalY + halfy);

      // if the final pos is not on 8
      }else{
        ystepper.runToNewPosition(finalY - halfy); // move up half square distance above target
        xstepper.runToNewPosition(finalX); // go to final X
        ystepper.runToNewPosition(finalY); // go to final Y
        servo.write(0);
        ystepper.runToNewPosition(finalY - halfy);
      }

    // if starting pos is on 8
    }else{
      
      // if ending pos is on 8
      if ( finalY == 0 ){
        ystepper.runToNewPosition(finalY + halfy); 
        xstepper.runToNewPosition(finalX);
        ystepper.runToNewPosition(finalY);
        servo.write(0);
        ystepper.runToNewPosition(finalY + halfy);
      // if ending pos is not on 8
      }else{
        ystepper.runToNewPosition(finalY - halfy); 
        xstepper.runToNewPosition(finalX);
        ystepper.runToNewPosition(finalY);
        servo.write(0);
        ystepper.runToNewPosition(finalY - halfy);
      }
    }
  }

  //drop the chess piece

  Serial.print("Done moving piece\n");
}

byte ReadOne165(){
  byte ret = 0x00;

  // The first one that is read is the highest bit (input D7 of the 74HC165).
  for (int i = 7; i >= 0; i--)
  {
    if (digitalRead(dataPin) == HIGH)
      bitSet(ret, i);

    digitalWrite(clockPin, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(clockPin, LOW);
  }

  return ret;
}

unsigned short myrand(void) {
  unsigned short r = (unsigned short)(seed % MYRAND_MAX);
  return r = ((r << 11) + (r << 7) + r) >> 1;
}
/* recursive minimax search */
/* (q,l)=window, e=current eval. score, */
/* E=e.p. sqr.z=prev.dest, n=depth; return score */
short D(short q, short l, short e, unsigned char E, unsigned char z, unsigned char n) {
  short m, v, i, P, V, s;
  unsigned char t, p, u, x, y, X, Y, H, B, j, d, h, F, G, C;
  signed char r;
  if (++Z > 30) {                                   /* stack underrun check */
    --Z; return e;
  }
  q--;                                          /* adj. window: delay bonus */
  k ^= 24;                                      /* change sides             */
  d = Y = 0;                                    /* start iter. from scratch */
  X = myrand() & ~M;                            /* start at random field    */
  W(d++ < n || d < 3 ||                         /* iterative deepening loop */
    z & K == I && (N < T & d < 98 ||            /* root: deepen upto time   */
                   (K = X, L = Y & ~M, d = 3)))                /* time's up: go do best    */
  { x = B = X;                                   /* start scan at prev. best */
    h = Y & S;                                   /* request try noncastl. 1st*/
    P = d < 3 ? I : D(-l, 1 - l, -e, S, 0, d - 3); /* Search null move         */
    m = -P < l | R > 35 ? d > 2 ? -I : e : -P;   /* Prune or stand-pat       */
    ++N;                                         /* node count (for timing)  */
    do {
      u = b[x];                                   /* scan board looking for   */
      if (u & k) {                                /*  own piece (inefficient!)*/
        r = p = u & 7;                             /* p = piece type (set r>0) */
        j = o[p + 16];                             /* first step vector f.piece*/
        W(r = p > 2 & r < 0 ? -r : -o[++j])        /* loop over directions o[] */
        { A:                                        /* resume normal after best */
          y = x; F = G = S;                         /* (x,y)=move, (F,G)=castl.R*/
          do {                                      /* y traverses ray, or:     */
            H = y = h ? Y ^ h : y + r;               /* sneak in prev. best move */
            if (y & M)break;                         /* board edge hit           */
            m = E - S & b[E] && y - E < 2 & E - y < 2 ? I : m; /* bad castling             */
            if (p < 3 & y == E)H ^= 16;              /* shift capt.sqr. H if e.p.*/
            t = b[H]; if (t & k | p < 3 & !(y - x & 7) - !t)break; /* capt. own, bad pawn mode */
            i = 37 * w[t & 7] + (t & 192);           /* value of capt. piece t   */
            m = i < 0 ? I : m;                       /* K capture                */
            if (m >= l & d > 1)goto C;               /* abort on fail high       */
            v = d - 1 ? e : i - p;                   /* MVV/LVA scoring          */
            if (d - !t > 1)                          /* remaining depth          */
            { v = p < 6 ? b[x + 8] - b[y + 8] : 0;    /* center positional pts.   */
              b[G] = b[H] = b[x] = 0; b[y] = u | 32;  /* do move, set non-virgin  */
              if (!(G & M))b[F] = k + 6, v += 50;     /* castling: put R & score  */
              v -= p - 4 | R > 29 ? 0 : 20;           /* penalize mid-game K move */
              if (p < 3)                              /* pawns:                   */
              { v -= 9 * ((x - 2 & M || b[x - 2] - u) + /* structure, undefended    */
                          (x + 2 & M || b[x + 2] - u) - 1  /*        squares plus bias */
                          + (b[x ^ 16] == k + 36))          /* kling to non-virgin King */
                     - (R >> 2);                       /* end-game Pawn-push bonus */
                V = y + r + 1 & S ? 647 - p : 2 * (u & y + 16 & 32); /* promotion or 6/7th bonus */
                b[y] += V; i += V;                     /* change piece, add score  */
              }
              v += e + i; V = m > q ? m : q;          /* new eval and alpha       */
              C = d - 1 - (d > 5 & p > 2 & !t & !h);
              C = R > 29 | d < 3 | P - I ? C : d;     /* extend 1 ply if in check */
              do
                s = C > 2 | v > V ? -D(-l, -V, -v,     /* recursive eval. of reply */
                                       F, 0, C) : v;    /* or fail low if futile    */
              W(s > q&++C < d); v = s;
              if (z && K - I && v + I && x == K & y == L) /* move pending & in root:  */
              { Q = -e - i; O = F;                     /*   exit if legal & found  */
                R += i >> 7; --Z; return l;            /* captured non-P material  */
              }
              b[G] = k + 6; b[F] = b[y] = 0; b[x] = u; b[H] = t; /* undo move,G can be dummy */
            }
            if (v > m)                               /* new best, update max,best*/
              m = v, X = x, Y = y | S & F;            /* mark double move with S  */
            if (h) {
              h = 0;  /* redo after doing old best*/
              goto A;
            }
            if (x + r - y | u & 32 |                 /* not 1st step,moved before*/
                p > 2 & (p - 4 | j - 7 ||             /* no P & no lateral K move,*/
                         b[G = x + 3 ^ r >> 1 & 7] - k - 6     /* no virgin R in corner G, */
                         || b[G ^ 1] | b[G ^ 2])               /* no 2 empty sq. next to R */
               )t += p < 5;                           /* fake capt. for nonsliding*/
            else F = y;                              /* enable e.p.              */
          } W(!t);                                  /* if not capt. continue ray*/
        }
      }
    } W((x = x + 9 & ~M) - B);                 /* next sqr. of board, wrap */
C: if (m > I - M | m < M - I)d = 98;           /* mate holds to any depth  */
    m = m + I | P == I ? m : 0;                  /* best loses K: (stale)mate*/
    if (z && d > 2)
    { *c = 'a' + (X & 7); c[1] = '8' - (X >> 4); c[2] = 'a' + (Y & 7); c[3] = '8' - (Y >> 4 & 7); c[4] = 0;
      char buff[150];
    }
  }                                             /*    encoded in X S,8 bits */
  k ^= 24;                                      /* change sides back        */
  --Z; return m += m < e;                       /* delayed-loss bonus       */
}

void gameOver() {
  for (;;);
}

void bkp() {
  for (int i = 0; i < 16 * 8 + 1; i++) {
    bk[i] = b[i];
  }
  Serial.print("BKP\n");
}

void detectCycle() {
  digitalWrite(latchPin, LOW);
  delayMicroseconds(pulseWidth);
  digitalWrite(latchPin, HIGH);
  uint64_t optionSwitch = 0;
  for (int i = 56; i>= 0; i-=8){
    optionSwitch |= ((uint64_t) ReadOne165()) << i;
  }
  for (int i=0; i<64; i++){
    if (bitRead(optionSwitch, i) != bitRead(oldOptionSwitch, i)){
        Serial.print(squares[i]);
    }
  }
  oldOptionSwitch = optionSwitch;
  Serial.print("cycle done\n");
  delay(300);
}

void setup() {
  Serial.begin(9600);
  // Chess Engine
  lastH[0] = 0;
  // Movement
  servo.attach(30);
  servo.write(0);
  xstepper.setMaxSpeed(1000.0);
  ystepper.setMaxSpeed(1000.0);
  xstepper.setAcceleration(500);
  ystepper.setAcceleration(500);
  pinMode(xstop, INPUT);
  pinMode(ystop, INPUT);
  
  // Square Detection
  pinMode(clockPin, OUTPUT);    // clock signal, idle LOW
  pinMode(latchPin, OUTPUT);    // latch (copy input into registers), idle HIGH
  digitalWrite(latchPin, HIGH);
  home();
  for (int i = 0; i < 5; i++){
    detectCycle();
  }
}

int myMoves[2] = {};
size_t moveCtr = 0;
void loop() {
  //detect human move
  digitalWrite(latchPin, LOW);
  delayMicroseconds(pulseWidth);
  digitalWrite(latchPin, HIGH);
  uint64_t optionSwitch = 0;
  for (int i = 56; i>= 0; i-=8){
    optionSwitch |= ((uint64_t) ReadOne165()) << i;
  }
  for (int i=0; i<64; i++){
    if (bitRead(optionSwitch, i) != bitRead(oldOptionSwitch, i)){
      // through the 64 loops, only one change should be found
      Serial.print(squares[i]);
      Serial.print(" :change found\n");

      if (moveCtr == 0){
        // piece pickup
        myMoves[0] = i;
        moveCtr++;
      } else if (moveCtr == 1){
        // piece put-down
        myMoves[1] = i;
        moveCtr = 0; // resetting
        // computer think move
        int r;
        // human move
        c[0] = squares[myMoves[0]][0];
        c[1] = squares[myMoves[0]][1];
        c[2] = squares[myMoves[1]][0];
        c[3] = squares[myMoves[1]][1];
        c[4] = 0;
        K = *c - 16 * c[1] + 799, L = c[2] - 16 * c[3] + 799; /* parse entered move */
        N = 0;
        T = 0x3F;                                 /* T=Computer Play strength */
        //bkp();                                    /* Save the board just in case */
        r = D(-I, I, Q, O, 1, 3);                 /* Check & do the human movement */
        if ( !(r > -I + 1) ) {
          Serial.println("Lose ");
          gameOver();
        }

        if (k == 0x10) {                          /* The flag turn must change to 0x08 */
          detectCycle();
          myMoves[0] = {'\0'};
          myMoves[1] = {'\0'};
          moveCtr = 0;
          Serial.println("No valid move");
          return;
        }else{

          strcpy(lastH, c);                         /* Valid human movement */

          mn++;                                     /* Next move */

          K = I;
          N = 0;
          T = 0x3F;                                 /* T=Computer Play strength */
          r = D(-I, I, Q, O, 1, 3);                 /* Think & do*/
          if ( !(r > -I + 1) ) {
            Serial.println("Lose*");
            gameOver();
          }

          strcpy(lastM, c);                         /* Valid ARDUINO movement */
          r = D(-I, I, Q, O, 1, 3);
          if ( !(r > -I + 1) ) {
            Serial.println(lastM);
            gameOver();
          }
          char first[3];
          first[0] = lastM[0];
          first[1] = lastM[1];
          first[2] = '\0';
          char second[3];
          second[0] = lastM[2];
          second[1] = lastM[3];
          second[2] = '\0';
          movePeice(first,second);
          servo.write(0);
          delay(1000);
          myMoves[0] = {};
          myMoves[1] = {};
          for (int i = 0; i < 5; i++){
            detectCycle();
          }
        }
      }
    }    
  }
  oldOptionSwitch = optionSwitch;
  delay(400);
}
