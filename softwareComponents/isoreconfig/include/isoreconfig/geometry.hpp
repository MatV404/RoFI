#include <array>
#include <cassert>
#include <vector>

#include <armadillo>

namespace rofi::isoreconfig {

constexpr double ERROR_MARGIN = 1.0 / 1000000;

/**
 * @brief "Point" (three-dimensional vector)
 */
using Point = arma::vec3;
/**
 * @brief "Cloud" of points
 */
using Cloud = std::vector< Point >;
/**
 * @brief Point transformation matrix
 */
using Transformation = arma::mat33;
/**
 * @brief "Position" (4x4 matrix)
 */
using Matrix = arma::mat44;


/**
 * @brief Converts a point to a position.
 */
Matrix pointToPos( const Point& point );

/**
 * @brief Converts a position to a point.
 */
Point posToPoint( const Matrix& position );

/**
 * @brief Calculates the centroid (unweighted average) 
 * of a given cloud of points.
 * Assumes the cloud is not empty.
 * @param cop Cloud to calculate the centroid from.
 * @return Unweighted average (centroid) of a given <cop>.
 */
Point centroid( const Cloud& cop );

/**
 * @brief Calculates the centroid (unweighted average) 
 * of a given vector of positions.
 * Assumes the vector is not empty.
 * @param positions Positions to calculate the centroid from.
 * @return Unweighted average (centroid) of positions 
 * given as matrices <positions>.
 */
Matrix centroid( const std::vector< Matrix >& positions );

/**
 * @brief Finds points furthest and second furthest away from <centerPos> 
 * in <cop>.
 * @param center Point to measure the distance from. 
 * @param cop Cloud of points to go through.
 * @param epsilon Maximum distance two points can be apart to be considered
 * the same point.
 * @return Array of two clouds, first contains points furthest away from 
 * <center>, second contains those second furthest away from <center>. 
 */
std::array< Cloud, 2 > longestVectors( 
    const Point& center, const Cloud& cop, 
    const double epsilon = ERROR_MARGIN );

/**
 * @brief Finds points furthest and second furthest away from the centroid
 * of <cop> in <cop>.
 * @param cop Cloud of points to go through.
 * @param epsilon Maximum distance two points can be apart to be considered
 * the same point.
 * @return Array of two clouds, first contains points furthest away from 
 * the center of gravity, second contains those second furthest away from it. 
 */
std::array< Cloud, 2 > longestVectors( 
    const Cloud& cop, const double epsilon = ERROR_MARGIN );

bool isometric( Cloud cop1, Cloud cop2 );

arma::mat cloudToScore( const Cloud& cop );
Cloud normalizedCloud( const Cloud& cop );
Cloud scoreToCloud( const arma::mat& score );

} // namespace rofi::isoreconfig
