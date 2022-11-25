#include <array>
#include <cassert>
#include <unordered_set>
#include <vector>

#include <armadillo>

#include <configuration/Matrix.h>

namespace rofi::isoreconfig {

constexpr double ERROR_MARGIN = 1.0 / 1000000;

/**
 * @brief Container of points as vectors
 */
using Points = std::vector< rofi::configuration::matrices::Vector >;
/**
 * @brief Set of points that can be normalized using PCA
 */
class Cloud;


/**
 * @brief Decides whether given clouds define an equal physical shape.
 * Assumes different number of points in cloud means different shapes.
 * Normalizes clouds using PCA and attempts to find an orthogonal transformation 
 * which transforms one cloud into the other.
 */
bool isometric( Cloud cop1, Cloud cop2 );


class Cloud
{
    arma::mat _score; // rows are points in cloud
    bool _normalized = false;
    arma::mat _coeff; // PC coefficients - cols are (base) eigenvectors
    arma::vec _latent; // eigenvalues of the covariance matrix of cloud
    arma::vec _tsquared; // Hotteling's statistic for each sample

    using Point = arma::subview_row<double>; 

public:

    explicit Cloud( const arma::mat& score ) : _score(score) {}
    explicit Cloud( arma::mat&& score ) : _score( std::move(score) ) {}
    explicit Cloud( const Points& pts ) 
    {
        _score.set_size( pts.size(), 3 );

        for ( size_t i = 0; i < pts.size(); ++i )
            _score.row(i) = { pts[i](0), pts[i](1), pts[i](2) };
    }

    Points toPoints() const
    {
        Points result( size() );

        for ( size_t i = 0; i < size(); ++i )
            result[i] = { _score(i, 0), _score(i, 1), _score(i, 2), 1 };
            
        return result;
    }

    /**
     * @brief Normalize the cloud to use its PCA coordinate system
     * (or its reflection in case the PCA transformation is a reflection,
     * so the shape of the cloud does not change).
     */
    void normalize()
    {
        arma::mat newScore;

        princomp( _coeff, newScore, _latent, _tsquared, _score );
        auto determinant = det( _coeff );
        assert( std::abs( determinant ) - 1 < ERROR_MARGIN );

        // If determinant is negative (therefore ~= -1), reflect along one plane (we use YZ).
        // (negative sign -> reflection, which can change the shape of the cloud)
        if ( determinant < 0 )
        {
            newScore *= arma::mat{ {-1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
            _coeff *= arma::mat{ {-1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
        } 

        _score = std::move( newScore );
        _normalized = true;
    }

    void transform( const arma::mat33& transf )
    {
        _normalized = false;
        _score *= transf;
    }
 
    bool isNormalized() const
    {
        return _normalized;
    }
    
    arma::mat transformation() const
    {
        if ( !_normalized )
            throw std::logic_error( "Cloud is not normalized" );

        // transposed _coeff is PCA transformation matrix
        return _coeff.t();
    }

    size_t size() const
    {
        return _score.n_rows;
    }

    bool operator==( const Cloud& o ) const
    {
        if ( size() != o.size() ) 
            return false;

        std::unordered_set< size_t > used;

        for ( size_t p1 = 0; p1 < size(); ++p1 )
        {
            auto [ cont, p2 ] = o.containsPoint( _score.row( p1 ), used );
            if ( !cont )
                return false;

            used.insert( p2 );
        }

        assert( used.size() == size() );
        return true;
    }

    auto begin() const
    {
        return _score.begin_row( 0 );
    }

    auto begin()
    {
        return _score.begin_row( 0 );
    }

    auto end() const
    {
        return _score.end_row( size() - 1 );
    }

    auto end()
    {
        return _score.end_row( size() - 1 );
    }

    void print() const
    {
        _score.print();
    }

private:

    std::pair< bool, size_t > containsPoint(
        const Point& p,
        const std::unordered_set< size_t >& used ) const
    {
        auto equalPoints = []( const Point& p1, const Point& p2 )
        {
            return std::abs( p1[0] - p2[0] ) < ERROR_MARGIN
                && std::abs( p1[1] - p2[1] ) < ERROR_MARGIN
                && std::abs( p1[2] - p2[2] ) < ERROR_MARGIN;
        }

        for ( size_t i = 0; i < size(); ++i )
            if ( !used.contains( i ) && equalPoints( p, _score.row(i) ) )
                return { true, i };
                
        return { false, 0 };
    }
};

} // namespace rofi::isoreconfig
