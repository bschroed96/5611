public class Vec3 {
  public float x, y, z;
  
  public Vec3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public String toString(){
    return "(" + x+ ", " + y +", " + z +")";
  }
  
  public float length(){
    return sqrt(x*x + y*y + z*z);
  }
  
  public float lengthSqr(){
    return length()*length();
  }
  
  public Vec3 plus(Vec3 rhs){
    return new Vec3(x + rhs.x, y + rhs.y, z + rhs.z);
  }
  
  public void add(Vec3 rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
  }
  
  public Vec3 minus(Vec3 rhs){
    return new Vec3(x - rhs.x, y - rhs.y, z - rhs.z);
  }
  
  public void subtract(Vec3 rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
  }
  
  public Vec3 times(float rhs){
    return new Vec3(x*rhs, y*rhs, z*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
    z *= rhs;
  }
  
  public Vec3 divide(float rhs){
    return new Vec3(x/rhs, y/rhs, z/rhs);
  }
  
  public void normalize(){
    float mag = length();
    x /= mag;
    y /= mag;
    z /= mag;
  }
  
  public Vec3 normalized(){
    float mag = length();
    return new Vec3(x/mag, y/mag, z/mag);
  }
  
  //If the vector is longer than maxL, shrink it to be maxL otherwise do nothing
  public void clampToLength(float maxL){
    float mag = length();
    if (length() > maxL) {
      x *= maxL/mag;
      y *= maxL/mag;
      z *= maxL/mag;
    }
  }
  
  //Grow or shrink the vector have a length of maxL
  public void setToLength(float newL){
    float mag = length();
    x *= newL/mag;
    y *= newL/mag;
    z *= newL/mag;
  }
  
  public float distanceTo(Vec3 rhs){
    return sqrt((x - rhs.x)*(x - rhs.x) + (y - rhs.y)*(y - rhs.y) + (z - rhs.z)*(z - rhs.z));
  }
}

Vec3 interpolate(Vec3 a, Vec3 b, float t){
  return a.plus(b.minus(a).times(t));
}

float dot(Vec3 a, Vec3 b){
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 cross(Vec3 a, Vec3 b){
  return new Vec3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}

Vec3 projAB(Vec3 a, Vec3 b){
  float adb = dot(a, b);
  return b.times(adb);
}
