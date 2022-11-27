#include <array>
#include <cassert>
#include <vector>
#include <queue>

#include <armadillo>

#include <configuration/rofiworld.hpp>

namespace rofi::isoreconfig {

using namespace rofi::configuration;
using VisitedId = size_t;
using Predecessors = std::unordered_map< VisitedId, VisitedId >;

bool equalConfig( const RofiWorld& bot1, const RofiWorld& bot2 );

class Shapes
{
    std::vector< RofiWorld > _visited;

public:
    Shapes() = default;

    bool contains( const RofiWorld& bot ) const
    {
        for ( const RofiWorld& found : _visited )
            if ( equalConfig( found, bot ) ) 
                return true;
            
        return false;
    }

    auto find( const RofiWorld& bot )
    {
        return find_if( _visited.begin(), _visited.end(), 
            [&]( const RofiWorld& found ){ return equalConfig( found, bot ); } );
    }

    VisitedId insert( const RofiWorld& bot )
    {
        _visited.push_back( bot );
        return _visited.size() - 1;
    }

    RofiWorld& operator[]( size_t i )
    {
        return _visited[i];
    }

    const RofiWorld& operator[]( size_t i ) const
    {
        return _visited[i];
    }

    size_t size() const
    {
        return _visited.size();
    }
};

class Reporter
{
    std::vector< size_t > _layerShapes;
    size_t _maxQueueSize = 0;
    size_t _configsTested = 0;
    size_t _shapesTotal = 0;
    size_t _shapesCurrent = 0;
    size_t _maxDescendants = 0;
    bool _pathFound = false;


public:
    Reporter() = default;

    void onUpdateVisited( const Shapes& )
    {
        ++_shapesTotal;
        ++_shapesCurrent;
    }

    void onUpdatePredecessors( const Predecessors& )
    {
        // pass
    }

    void onUpdateDistance( const std::unordered_map< VisitedId, size_t >& )
    {
        // pass
    }

    void onNewDescendant( const RofiWorld& )
    {
        ++_configsTested;
    }

    void onUpdateLayer( size_t layer )
    {
        assert( layer == _layerShapes.size() );
        _layerShapes.push_back( _shapesCurrent );
        _shapesCurrent = 0;
    }

    void onUpdateQueue( const std::queue< VisitedId >& bfsQueue )
    {
        _maxQueueSize = std::max( _maxQueueSize, bfsQueue.size() );
    }

    void onGenerateDescendants( const std::vector< RofiWorld >& desc )
    {
        _maxDescendants = std::max( _maxDescendants, desc.size() );
    }

    void onReturn( bool pathFound )
    {
        _pathFound = pathFound;
    }

    std::string toString() const
    {
        std::stringstream result;
        
        result << "foundPath: "          << (_pathFound ? "1" : "0") << "\n";
        result << "currentLayer: "       << _layerShapes.size()      << "\n";
        result << "currentLayerShapes: " << _shapesCurrent           << "\n";
        result << "totalShapes: "        << _shapesTotal             << "\n";
        result << "maxQSize: "           << _maxQueueSize            << "\n";
        result << "maxDescendants: "     << _maxDescendants          << "\n";
        result << "allConfigs: "         << _configsTested           << "\n";
        result << "layerShapes: ";
        for ( size_t shapeCount : _layerShapes ) 
            result << shapeCount << ";";
        result << "\n";

        return result.str();
    }
};

std::vector<rofi::configuration::RofiWorld> bfsShapes(
    const rofi::configuration::RofiWorld& start, 
    const rofi::configuration::RofiWorld& target,
    float step, Reporter& rep );

} // namespace rofi::isoreconfig
