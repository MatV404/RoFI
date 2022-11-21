#include <isoreconfig/geometry.hpp>

namespace rofi::isoreconfig {

Transformation rotate( double r, const Point &p )
{
    const int x = 0, y = 1, z = 2;
    Transformation rotate = arma::eye(3,3);
    auto cos_r = cos(r); auto sin_r = sin(r);

    rotate(0,0) = cos_r + p(x) * p(x) * (1 - cos_r);
    rotate(0,1) = p(x) * p(y) * (1 - cos_r) - p(z) * sin_r;
    rotate(0,2) = p(x) * p(z) * (1 - cos_r) + p(y) * sin_r;

    rotate(1,0) = p(x) * p(y) * (1 - cos_r) + p(z) * sin_r;
    rotate(1,1) = cos_r + p(y) * p(y) * (1 - cos_r);
    rotate(1,2) = p(y) * p(z) * (1 - cos_r) - p(x) * sin_r;

    rotate(2,0) = p(z) * p(x) * (1 - cos_r) - p(y) * sin_r;
    rotate(2,1) = p(z) * p(y) * (1 - cos_r) + p(x) * sin_r;
    rotate(2,2) = cos_r + p(z) * p(z) * (1 - cos_r);

    return rotate;
}

void ceilPoint ( Point& p )
{ 
    for ( int i = 0; i < 3; ++i ) 
        p(i) = std::floor( p(i) / ERROR_MARGIN + 0.5 ) * ERROR_MARGIN; 
}

Matrix pointToPos( const Point& point )
{
    Matrix result( arma::fill::eye );
    for ( int i = 0; i < 3; ++i )
        result(i,3) = point(i);
    return result;
}

Point posToPoint( const Matrix& position )
{
    return Point{ position.at(0,3), position.at(1,3), position.at(2,3) };
}

Point centroid( const Cloud& cop )
{
    assert( cop.size() >= 1 );
    Point result = { 0, 0, 0 };
    
    for ( const Point& point : cop )
        result += point;

    double pointCount = cop.size();
    return result / Point( { pointCount, pointCount, pointCount } );
}

Matrix centroid( const std::vector< Matrix >& positions )
{
    Cloud cop;
    for ( const Matrix& pos : positions )
        cop.push_back( posToPoint( pos ) );

    return pointToPos( centroid( cop ) );
}

double cubeNorm( const Point& vec )
{
    return vec( 0 ) * vec( 0 ) + vec( 1 ) * vec( 1 ) + vec( 2 ) * vec( 2 );
} 

std::array< Cloud, 2 > longestVectors( const Point& center, const Cloud& cop, const double epsilon )
{
    std::array< Cloud, 2 > result{ std::vector{center} };
    double longestNorm = 0;
    double sndLongestNorm = 0;

    for ( const Point& currPoint : cop )
    {
        double nextNorm = cubeNorm( currPoint - center );

        if ( nextNorm - longestNorm > epsilon )
        {
            // New longest vector, shift old longest to second longest
            result[1] = std::exchange( result[0], { currPoint } );
            sndLongestNorm = std::exchange( longestNorm, nextNorm );
        }
        else if ( std::abs( nextNorm - longestNorm ) <= epsilon )
            // Another longest vector
            result[0].push_back( currPoint );
        else if ( nextNorm - sndLongestNorm > epsilon )
        {
            // New second longest vector
            result[1] = { currPoint };  
            sndLongestNorm = nextNorm;
        }
        else if ( std::abs( nextNorm - sndLongestNorm ) <= epsilon )
            // Another second longest vector
            result[1].push_back( currPoint );         
    }

    return result;
}

std::array< Cloud, 2 > longestVectors( const Cloud& cop, const double eps )
{
    return longestVectors( centroid( cop ), cop, eps );
}

arma::mat cloudToScore( const Cloud& cop )
{
    arma::mat result( cop.size(), 3 );

    for ( size_t i = 0; i < cop.size(); ++i )  
        for ( size_t j = 0; j < 3; ++j )
            result(i, j) = cop[i](j);

    return result;
}

Cloud scoreToCloud( const arma::mat& score )
{
    Cloud result;

    for ( size_t i = 0; i < score.n_rows; ++i )
        result.push_back( { score(i,0), score(i, 1), score(i, 2) } );

    return result;
}

Cloud normalizedCloud( Cloud cop, const Point& center )
{
    for ( Point& p : cop ) p -= center;
    return cop;
}

Cloud normalizedCloud( const Cloud& cop )
{
    return normalizedCloud( cop, centroid( cop ) );
}

bool equalPoints( const Point& p1, const Point& p2 )
{
    return std::abs( p1(0) - p2(0) ) < ERROR_MARGIN 
        && std::abs( p1(1) - p2(1) ) < ERROR_MARGIN 
        && std::abs( p1(2) - p2(2) ) < ERROR_MARGIN;
}

bool pointInCloud( const Point& p, const Cloud& cop )
{
    for ( size_t i = 0; i < cop.size(); ++i ) 
        if ( equalPoints( p, cop[i] ) )
            return true;
    return false;
}

bool equalClouds( const Cloud& cloud1, const Cloud& cloud2 )
{   
    if ( cloud1.size() != cloud2.size() ) return false;

    for ( size_t i = 0; i < cloud1.size(); ++i ) 
    {
        if ( !pointInCloud( cloud1[i], cloud2 ) )
            return false;
    }
        
    return true; 
}

/* bool equalClouds( const Cloud& cloud1, const Cloud& cloud2 )
{   
    if ( cloud1.size() != cloud2.size() ) return false;

    for ( size_t i = 0; i < cloud1.size(); ++i ) 
        if ( !equalPoints( cloud1[i], cloud2[i] ) )
            return false;
    return true; 
} */

bool isometric( Cloud cop1, Cloud cop2 )
{
    if ( cop1.size() != cop2.size() ) return false;

    // TODO Do we need to normalize or does PCA do it for us?
    // PCA normalizes the set
    // cop1 = normalizeCloud( cop1 );
    // cop2 = normalizeCloud( cop2 );

    // Sorting by value of elements
    auto comparePoints = [&]( const Point& p1, const Point& p2 ) 
        { return p1(0) < p2(0) || ((p1(0) == p2(0) && p1(1) < p2(1)) || (p1(0) == p2(0) && p1(1) == p2(1) && p1(2) < p2(2))); };

    arma::mat coeff1, coeff2; // principal component coefficients
    arma::mat score1, score2; // projected data - points in new coordinate system
    arma::vec latent1, latent2; // eigenvalues of the covariance matrix of X - eigenvectors for PCA space
    arma::vec tsquared1, tsquared2; // Hotteling's statistic for each sample

    princomp( coeff1, score1, latent1, tsquared1, cloudToScore( cop1 ) );
    princomp( coeff2, score2, latent2, tsquared2, cloudToScore( cop2 ) );

    auto det1 = det(coeff1);
    auto det2 = det(coeff2);
    assert( std::abs(det1) - 1 < ERROR_MARGIN );
    assert( std::abs(det2) - 1 < ERROR_MARGIN );

    if ( det1 * det2 < 0 ) 
        score1 *= arma::mat{ {-1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

    cop1 = scoreToCloud( score1 );
    cop2 = scoreToCloud( score2 );

    std::vector< Point > axes = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
    std::vector< double > half_space = { 0, M_PI };
    std::vector< double > rotations = { 0, M_PI_2, M_PI, M_PI + M_PI_2 };

    // TODO only some rotations are required - X should be X or -X, not +-Y or +-Z
    // but we mirror along X, which might change things
    for ( size_t ax = 0; ax < axes.size(); ++ax )
        for ( const auto& hp : half_space )
            for ( const auto& alpha : rotations )
            {
                auto rot = rotate( hp, axes[ (ax + 1) % axes.size() ] ) * rotate( alpha, axes[ax] );

                auto cop1transformed = cop1;

                for ( Point& p : cop1transformed ) p = rot * p;

                // Both clouds should contain the same points (with some tolerance)
                if ( equalClouds( cop1transformed, cop2 ) )
                    return true;
            }

    return false;
}

} // namespace rofi::isoreconfig
