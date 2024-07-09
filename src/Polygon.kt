open class Polygon(private var points: MutableList<Double>) {
    override fun toString(): String {
        return points.toString()
    }
}

class Field(points: MutableList<Double>): Polygon(points) {

}

class Obstacle(points: MutableList<Double>): Polygon(points) {

}
