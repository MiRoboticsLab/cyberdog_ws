#ifndef _CIRC_BUF_H
#define _CIRC_BUF_H 1

// size must be power - of - 2.
struct circ_buf
{
  circ_buf(int _size, char * _buf)
    : buf(_buf), head(0), tail(0), size(_size) {
  }
  int cnt() {return (head - tail) & (size - 1);}
  int space() {return (tail - (head + 1)) & size;}
  void pushc(char d)
  {
    int tmp = head;  buf[tmp] = d; head = (tmp + 1) & (size - 1);
  }
  char popc()
  {
    int tmp = tail;  tail = (tail + 1) & (size - 1); return buf[tmp];
  }
  void reset() {head = tail = 0;}

  char * buf;
  int head;
  int tail;
  int size;
};
#endif /* _CIRC_BUF_H  */
