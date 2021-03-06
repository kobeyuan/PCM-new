#ifndef COORD_H
#define COORD_H


struct Coord
{
  int x, y;
  
  Coord() {}
  Coord(int a, int b) { x = a; y = b; }
  
  Coord operator- ()        { return Coord(-x, -y); }
  Coord operator+ (Coord a) { return Coord(x + a.x, y + a.y); }
  Coord operator- (Coord a) { return Coord(x - a.x, y - a.y); }
  bool  operator< (Coord a) { return (x <  a.x) && (y <  a.y); }
  bool  operator<=(Coord a) { return (x <= a.x) && (y <= a.y); }
  bool  operator> (Coord a) { return (x >  a.x) && (y >  a.y); }
  bool  operator>=(Coord a) { return (x >= a.x) && (y >= a.y); }
  bool  operator==(Coord a) { return (x == a.x) && (y == a.y); }
  bool  operator!=(Coord a) { return (x != a.x) || (y != a.y); }
  void operator-=(Coord a) { x-=a.x; y-=a.y;}
};
//#define IMREF(im, p, c) (im[3*((p.y)*_w + (p.x)) + (c)]);
#define IMREF(im, p) (imRef((im), (p).x, (p).y))


#endif
