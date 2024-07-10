import Polygon

class CostMatrix(grid: Grid,
                 var matrix_size = Grid.obstacles_points.size + Grid.field_points.size):
Array<DoubleArray>(matrix_size)
{
    val emptyArray = arrayOf<DoubleArray>()

    fun calcCost() {

    }
}

class HamiltonPath(cost_matrix: CostMatrix)  {

    fun findPath(): List<Point> {

    }
}