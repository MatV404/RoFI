#include <isoreconfig/geometry.hpp>

namespace rofi::isoreconfig {

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

Point centroid( const std::vector< Matrix >& positions )
{
    Cloud cop;
    for ( const Matrix& pos : positions )
        cop.push_back( posToPoint( pos ) );

    return centroid( cop );
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

Cloud normalizeCloud( Cloud cop, const Point& center )
{
    for ( Point& p : cop ) p -= center;
    return cop;
}

Cloud normalizeCloud( const Cloud& cop )
{
    return normalizeCloud( cop, centroid( cop ) );
}

bool equalPoints( const Point& p1, const Point& p2 )
{
    // measure precision
    // std::cout << std::abs( p1(0) - p2(0) ) << " " << std::abs( p1(1) - p2(1) ) << " " << std::abs( p1(2) - p2(2) ) << "\n";
    return std::abs( p1(0) - p2(0) ) < ERROR_MARGIN 
        && std::abs( p1(1) - p2(1) ) < ERROR_MARGIN 
        && std::abs( p1(2) - p2(2) ) < ERROR_MARGIN;
}

// bool equalClouds( const Cloud& cloud1, const Cloud& cloud2 )
// {   
//     if ( cloud1.size() != cloud2.size() ) return false;

//     for ( size_t i = 0; i < cloud1.size(); ++i ) 
//         if ( !equalPoints( cloud1[i], cloud2[i] ) )
//             return false;
//     return true; 
// }

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
        // cloud1[i].print();
        if ( !pointInCloud( cloud1[i], cloud2 ) )
            return false;
    }
        
    return true; 
}

bool isometric( Cloud cop1, Cloud cop2, bool normalize )
{
    if ( cop1.size() != cop2.size() ) return false;

    if ( normalize )
    {
        cop1 = normalizeCloud( cop1 );
        cop2 = normalizeCloud( cop2 );
    }

    // Sorting by value of elements
    auto comparePoints = [&]( const Point& p1, const Point& p2 ) 
        { return p1(0) < p2(0) || ((p1(0) == p2(0) && p1(1) < p2(1)) || (p1(0) == p2(0) && p1(1) == p2(1) && p1(2) < p2(2))); };

    // std::sort( cop1.begin(), cop1.end(), comparePoints );
    // std::sort( cop2.begin(), cop2.end(), comparePoints );

    // std::cout << "cop1:\n";
    // cloudToScore( cop1 ).print();

    // std::cout << "cop2:\n";
    // cloudToScore( cop2 ).print();

    // Cloud should have its center in centroid
    // assert( cubeNorm(centroid(cop1)) < ERROR_MARGIN );
    // assert( cubeNorm(centroid(cop2)) < ERROR_MARGIN );

    // auto comparePoints = [&]( const Point& p1, const Point& p2 ) 
    //     { return cubeNorm( p1 ) < cubeNorm( p2 ); };

    arma::mat coeff1, coeff2; // principal component coefficients
    arma::mat score1, score2; // projected data
    arma::vec latent1, latent2; // eigenvalues of the covariance matrix of X
    arma::vec tsquared1, tsquared2; // Hotteling's statistic for each sample

    princomp( coeff1, score1, latent1, tsquared1, cloudToScore( cop1 ) );
    princomp( coeff2, score2, latent2, tsquared2, cloudToScore( cop2 ) );

    cop1 = scoreToCloud( score1 );
    cop2 = scoreToCloud( score2 );

    // std::cout << "score1:\n";
    // score1.print();
    // std::cout << "score2:\n";
    // score2.print();

    // Round to some precision
    auto ceilPoint = [&]( Point& p )
        { for ( int i = 0; i < 3; ++i ) p(i) = std::floor( p(i) / ERROR_MARGIN + 0.5 ) * ERROR_MARGIN; };

    // std::for_each( cop1.begin(), cop1.end(), ceilPoint );
    // std::for_each( cop2.begin(), cop2.end(), ceilPoint );

    // Sort by size of coordinates - fixed reference cloud
    // std::sort( cop2.begin(), cop2.end(), comparePoints );

    // Try all mirrored coordinate systems (for symmetric shapes)
    for ( const Point& trans : std::vector< Point >({ 
        { 1, 1, 1 }, { -1, 1, 1 }, 
        { 1, -1, 1 }, { 1, 1, -1 }, 
        { -1, -1, 1 }, { -1, 1, -1 }, 
        { 1, -1, -1 }, { -1, -1, -1 } }))
    {
        for ( Point& p : cop1 ) for ( int i = 0; i < 3; ++i ) p(i) *= trans(i);
        // std::for_each( cop1.begin(), cop1.end(), ceilPoint );
        
        // std::sort( cop1.begin(), cop1.end(), comparePoints );
        // std::sort( cop2.begin(), cop2.end(), comparePoints );

        // std::cout << "cop1:\n";
        // cloudToScore( cop1 ).print();

        // std::cout << "cop2:\n";
        // cloudToScore( cop2 ).print();

        // Both clouds should contain the same points (with some tolerance)
        if ( equalClouds( cop1, cop2 ) )
            return true;
    }

    // comparing matrices
    // arma::approx_equal( score1, score2, "absdiff", ERROR_MARGIN );

    return false;
}

} // namespace rofi::isoreconfig
