#define __ASSERT_USE_STDERR

#include <assert.h>

float fixangle(float angle, float reference) {
  float out = angle;
  if(reference < angle) {
    for(; abs(out-reference-360.0) < abs(out-reference); out -= 360.0);
    return out;
  } else {
    for(; abs(out-reference+360.0) < abs(out-reference); out += 360.0);
    return out;
  }
}

struct Vector3D {
    union {float x, roll;};
    union {float y, pitch;};
    union {float z, yaw;};
    
    Vector3D() {
        x = 0;
        y = 0;
        z = 0;
    }

    Vector3D(float x1,float y1,float z1=0) {
        x = x1;
        y = y1;
        z = z1;
    }

    /*Vector3D(float v[3]) {
        vec=v;
    }*/

    Vector3D(const Vector3D &v) { //copy constructor
      x = v.x;
      y = v.y;
      z = v.z;
    }
    
    Vector3D operator+(const Vector3D &v) {    //addition
      return Vector3D(x+v.x,y+v.y,z+v.z);
    }
    
    Vector3D &operator+=(const Vector3D &v) {  //assigning new result to the vector
      x+=v.x;
      y+=v.y;
      z+=v.z;
      return *this;
    }
    
    Vector3D operator-(const Vector3D &v) {    //substraction
      return Vector3D(x-v.x,y-v.y,z-v.z);
    }
    
    Vector3D &operator-=(const Vector3D &v) {  //assigning new result to the vector
      x-=v.x;
      y-=v.y;
      z-=v.z;
      return *this;
    }
    
    Vector3D operator*(float value) {    //multiplication
      return Vector3D(x*value,y*value,z*value);
    }
    
    Vector3D &operator*=(float value) {  //assigning new result to the vector.
      x*=value;
      y*=value;
      z*=value;
      return *this;
    }
    
    Vector3D operator/(float value) {    //division
      assert(value!=0);
      return Vector3D(x/value,y/value,z/value);
    }
    
    Vector3D &operator/=(float value) {  //assigning new result to the vector
      assert(value!=0);
      x/=value;
      y/=value;
      z/=value;
      return *this;
    }
    
    Vector3D &operator=(const Vector3D &v) {
      x = v.x;
      y = v.y;
      z = v.z;
      return *this;
    }
    
    float dot_product(const Vector3D &v) { //scalar dot_product
      return x*v.x+v.y*y+v.z*z;
    }
    
    Vector3D cross_product(const Vector3D &v) {    //cross_product
      return Vector3D(y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x);
    }
    
    float square() { //gives square of the vector
      return x*x+y*y+z*z;
    }
    
    float magnitude() {  //magnitude of the vector
      return sqrt(square());
    }
    
    Vector3D normalize() {   //normalized vector
      float m = magnitude();
      assert(m!=0);
      *this/=m;
      return *this;
    }

    float distance(const Vector3D &v) {    //gives distance between two vectors
      Vector3D dist=*this-v;
      return dist.magnitude();
    }

    Vector3D componentMultiply(float a, float b, float c) {
      return Vector3D(x*a,y*b,z*c);
    }

    void fix(Vector3D ref) {
      yaw = fixangle(yaw, ref.yaw);
      roll = fixangle(roll, roll);
      pitch = fixangle(pitch, pitch);
    }

    Vector3D quaternionRotate(float qw, float qx, float qy, float qz) {
       Vector3D out;
       out.x = 2.0*((0.5-(qy*qy+qz*qz))*x + (qx*qy-qz*qw)*y + (qx*qz+qy*qw)*z);
       out.y = 2.0*((qx*qy+qz*qw)*x + (0.5-(qx*qx+qz*qz))*y + (qy*qz-qx*qw)*z);
       out.z = 2.0*((qx*qz-qy*qw)*x + (qy*qz+qx*qw)*y + (0.5-(qx*qx+qy*qy))*z);
       return out;
    }

    void print() {
      Serial.print(x);
      Serial.print(", ");
      Serial.print(y);
      Serial.print(", ");
      Serial.print(z);
    }

    void println() {
      print();
      Serial.println();
    }
};
