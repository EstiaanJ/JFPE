package xyz.ejvr.physics;

public record VectorDouble(double x, double y) {

    public  VectorDouble(VectorDouble vec){
        this(vec.x,vec.y);
    }
    public double diffX(VectorDouble p){
        return x - p.x;
    }
    public double diffY(VectorDouble p){
        return y - p.y;
    }

    public double radiusSquared(){
        return (x*x) + (y*y);
    }

    public double radius() {
        return Math.sqrt(radiusSquared());
    }

    public double angle() {
        return Math.atan2(y,x);
    }

    public float xFloat(){
        return (float) x;
    }

    public float yFloat(){
        return (float) y;
    }

    public VectorDouble rotateAboutOrigin(double angle){
        return new VectorDouble(rotatePoint(0,0,angle,this));
    }


    public boolean equals(VectorDouble p) {
        return this.x == p.x && this.y == p.y;
    }

    public double distanceBetween(VectorDouble point){
        double xFinal = this.x - point.x;
        double yFinal = this.y - point.y;
        return Math.sqrt((xFinal * xFinal) + (yFinal * yFinal));
    }


    public double dotProduct(VectorDouble vec) {
        return x * vec.x + y * vec.y;
    }

    public double crossProduct2D(VectorDouble vec) {
        return (x * vec.y) - (y * vec.x);
    }

    public VectorDouble add(VectorDouble vec) {
        return new VectorDouble(x + vec.x, y + vec.y);
    }

    public VectorDouble negate(){
        return new VectorDouble(-x,-y);
    }

    public VectorDouble sub(VectorDouble vec) {
        return this.add(vec.negate());
    }

    public VectorDouble tangent() {
        return new VectorDouble(-y,x);
    }

    public VectorDouble scale(double scalar) {
        return new VectorDouble(x * scalar,y * scalar);
    }

    public VectorDouble normalize() {
        return scale(1 / radius());
    }

    public String toString(){
        return ("r: " + radius() + " x: " + x + " y: " + y);
    }

    /*
    Adapted from:
    twe4ked, Nils Pipenbrinck
    Stack Overflow 2022
    https://stackoverflow.com/questions/2259476/rotating-a-point-about-another-point-2d
     */
    public static VectorDouble rotatePoint(double cx, double cy, double angle, VectorDouble p)
    {
        double newX;
        double newY;

        double sine = Math.sin(angle);
        double cosine = Math.cos(angle);

        // translate point to origin:
        newX = p.x - cx;
        newY = p.y - cy;
        //VectorDouble returnPoint = new VectorDouble(p.x - cx,p.y - cy);

        // rotate point
        newX = newX * cosine - newY * sine;
        newY = newX * sine + newY * cosine;

        return new VectorDouble(newX + cx, newY + cy);
    }
}