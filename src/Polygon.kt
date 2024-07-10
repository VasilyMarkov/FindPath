data class Point(val x: Double, val y: Double)

data class Line(val p1: Point, val p2: Point)

open class Polygon(private var points: List<Point>) {
    override fun toString(): String {
        return points.toString()
    }
}

class Field(points: List<Point>): Polygon(points) {
    init {

    }
}

class Obstacle(points: List<Point>): Polygon(points) {
    enum class Category {
        A, B, C, D, NONE
    }

    var category = Category.NONE

    init {

    }

    fun categorize() {

    }

}

class Grid(grid_step: Double, azimut: Double, field: Field, obstacles: List<Obstacle>) {

    val obstacles_points = HashMap<UInt, List<Point>>()
    val field_points = emptyList<Point>()

    init {
        generateLines()
    }

    private fun generateLines() {

    }

    //Get points of intersection and categorize
    private fun intersection(obs: Obstacle) {

    }

    //Get points of intersection
    private fun intersection(field: Field) {

    }

    fun generateGlobalPoints(field: Field) {
        intersection(field)
    }

    fun generateObstaclesPoints(obstacles: List<Obstacle>) {
        for (obs in obstacles) {
            intersection(obs)
        }
    }
}