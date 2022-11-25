#include <isoreconfig/geometry.hpp>
#include <configuration/Matrix.h>

namespace rofi::isoreconfig {

arma::mat33 rotate( double r, const arma::vec3 &p )
{
    const int x = 0, y = 1, z = 2;
    arma::mat33 result = arma::eye(3,3);
    auto cos_r = cos(r); auto sin_r = sin(r);

    result(0,0) = cos_r + p(x) * p(x) * (1 - cos_r);
    result(0,1) = p(x) * p(y) * (1 - cos_r) - p(z) * sin_r;
    result(0,2) = p(x) * p(z) * (1 - cos_r) + p(y) * sin_r;

    result(1,0) = p(x) * p(y) * (1 - cos_r) + p(z) * sin_r;
    result(1,1) = cos_r + p(y) * p(y) * (1 - cos_r);
    result(1,2) = p(y) * p(z) * (1 - cos_r) - p(x) * sin_r;

    result(2,0) = p(z) * p(x) * (1 - cos_r) - p(y) * sin_r;
    result(2,1) = p(z) * p(y) * (1 - cos_r) + p(x) * sin_r;
    result(2,2) = cos_r + p(z) * p(z) * (1 - cos_r);

    return result;
}

bool isometric( Cloud cop1, Cloud cop2 )
{
    // Assume different number of points implies nonequal shapes
    // (even extra connection point results in a different shape)
    if ( cop1.size() != cop2.size() )
        return false;

    cop1.normalize();
    cop2.normalize();

    std::vector< arma::vec3 > axes = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
    std::vector< double > rotations = { 0, M_PI_2, M_PI, M_PI + M_PI_2 };

    // Go through 12 orthogonal rotations in third dimension
    // (4 out of 8 octants, each of which can have the axes arranged in 3 ways)
    for ( size_t ax = 0; ax < axes.size(); ++ax )
        for ( const auto& alpha : rotations )
        {
            Cloud cop1transformed = cop1;
            cop1transformed.transform( rotate( alpha, axes[ax] ) );

            if ( cop1transformed == cop2 )
                return true;
        }

    return false;
}

} // namespace rofi::isoreconfig
