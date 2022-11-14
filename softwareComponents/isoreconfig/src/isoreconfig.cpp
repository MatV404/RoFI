#include <cassert>
#include <queue>

#include <isoreconfig/isomorphic.hpp>
#include <isoreconfig/isoreconfig.hpp>
#include <configuration/serialization.hpp>

using VisitedId = size_t;

namespace rofi::isoreconfig {

using namespace rofi::configuration;


bool equalConfig( const RofiWorld& bot1, const RofiWorld& bot2 )
{
    return equalShape( bot1, bot2, true );
}

void saveToFile( const RofiWorld& bot, const std::string& path )
{
    auto jason = serialization::toJSON( bot );
    std::ofstream out(path);
    // std::cout << jason.dump();
    out << jason.dump();
    out.close();
}

class VisitedShapes
{
    std::vector< RofiWorld > _visited;

public:
    VisitedShapes() = default;

    bool contains( const RofiWorld& bot ) const
    {
        std::stringstream path;
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/bot.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( bot, path.str() );

        for ( const RofiWorld& found : _visited )
        {
            path.str("");
            path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/found.json";
            std::cout << "writing to " << path.str() << "\n";
            saveToFile( found, path.str() );

            if ( equalConfig( found, bot ) ) 
                return true;
        }
            
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
};

void generateRotationsRec( std::vector< std::vector< float > >& result, std::vector< float >& current,
     const std::array< float, 3 >& possChange, size_t toBeAdded )
{
    if ( toBeAdded == 0 ) 
    {
        result.push_back( current );
        return;
    }

    for ( float change : possChange )
    {
        current.push_back( change );
        generateRotationsRec( result, current, possChange, toBeAdded - 1 );
        assert( current.size() > 0 );
        current.pop_back();
    }
}

std::vector< std::vector< float > > generateRotations( size_t dog, float step ) 
{
    std::vector< std::vector< float > > result;
    std::array< float, 3 > possChange { -step, step, 0 };
    std::vector< float > current;

    generateRotationsRec( result, current, possChange, dog );
    assert( result.back() == std::vector< float >( dog, 0 ) );
    result.pop_back(); // Remove identity
    return result;
}

// Get possible configurations made from the current one
    // "1 step" away, (TODO ignoring configurations of identical classes?)
std::vector< RofiWorld > getDescendants(
    const RofiWorld& current, float step, unsigned int bound ) 
{
    std::stringstream path;
    path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/parent.json";
    std::cout << "writing to " << path.str() << "\n";
    saveToFile( current, path.str() );

    std::vector< RofiWorld > result;

    // Generate all possible new connections from mod-comp to mod-comp - TODO doesnt connect on triple
    std::vector< std::tuple< ModuleId, int, ModuleId, int, roficom::Orientation > > possConnect;
    for ( const auto& modInf : current.modules() )
    {
        assert( modInf.module->type == ModuleType::Universal );

        for ( const auto& conn : modInf.module->connectors() )
        {
            assert( conn.type == ComponentType::Roficom );
            std::optional<std::pair< const Component&, roficom::Orientation >> poss = conn.getNearConnector();
            if ( poss ) 
            {
                assert( poss->first.parent );
                ModuleId mod1 = modInf.module->getId();
                int comp1 = conn.getIndexInParent();
                ModuleId mod2 = poss->first.parent->getId();
                int comp2 = poss->first.getIndexInParent();
                auto newConn = std::tie( mod1, comp1, mod2, comp2, poss->second );
                possConnect.push_back( newConn );
            } 
        }
    }

    // filter duplicates
    std::vector< std::tuple< ModuleId, int, ModuleId, int, roficom::Orientation > > noDupes;
    for ( const auto& conn1 : possConnect )
    {
        // Assert that a connector cannot be connected to itself
        // possibly check just module ids, since universal module cannot connect itself to itself
        assert( std::get<0>( conn1 ) != std::get<2>( conn1 ) 
            || std::get<1>( conn1 ) != std::get<3>( conn1 ) );

        bool add = true;
        for ( const auto& conn2 : noDupes )
        {
            // Assert that every connector can be connected only to one other connector
            assert( std::get<0>( conn1 ) != std::get<0>( conn2 ) 
                || std::get<1>( conn1 ) != std::get<1>( conn2 ) );
            
            if ( std::get<0>( conn1 ) == std::get<2>( conn2 ) 
                && std::get<1>( conn1 ) == std::get<3>( conn2 ) )
            {
                add = false;
                break;
            }
        }
        if ( add ) noDupes.push_back( conn1 );
    }
    possConnect = std::move( noDupes );

    // generate new rofibots by connecting possible roficoms - TODO includes already connected?
    for ( const auto& conn : possConnect )
    {
        RofiWorld newBot = current;
        connect( 
            newBot.getModule( std::get<0>( conn ) )->components()[ std::get<1>( conn ) ], 
            newBot.getModule( std::get<2>( conn ) )->components()[ std::get<3>( conn ) ],
            std::get<4>( conn ) );
        assert( current.isPrepared() );
        
        bool consistent = true;
        try { newBot.prepare(); } catch( std::runtime_error& e ) { consistent = false; }
        if ( consistent ) result.push_back( newBot );

        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/newBot.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( newBot, path.str() );

        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/parent.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( current, path.str() );
    }

    // generate new rofibots by disconnecting possible roficoms
    auto allConnects = current.roficomConnections();
    for ( auto start = allConnects.begin(); start != allConnects.end(); ++start )
    {
        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/parent.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( current, path.str() );

        RofiWorld newBot = current;
        newBot.disconnect( start.get_handle() );
        assert( current.isPrepared() );

        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/newBot.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( newBot, path.str() );

        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/parent.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( current, path.str() );

        bool consistent = true;
        try { newBot.prepare(); } catch( std::runtime_error& ) { consistent = false; }
        if ( consistent ) result.push_back( newBot );
    }

    // Turn TODO
    for ( const auto& modInf : current.modules() )
    {
        for ( size_t j = 0; j < modInf.module->joints().size(); ++j )
        {
            // TODO generate rotations for all reocurring dogs in advance
            for ( auto& possRot : generateRotations( modInf.module->joints()[j].joint->positions().size(), step ) )
            {
                path.str("");
                path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/parent.json";
                std::cout << "writing to " << path.str() << "\n";
                saveToFile( current, path.str() );

                RofiWorld newBot = current;
                // Shouldnt work - is const
                newBot.getModule(  modInf.module->getId() )->joints()[j].joint->changePositions( possRot );
                assert( current.isPrepared() );

                path.str("");
                path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/newBot.json";
                std::cout << "writing to " << path.str() << "\n";
                saveToFile( newBot, path.str() );

                path.str("");
                path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/parent.json";
                std::cout << "writing to " << path.str() << "\n";
                saveToFile( current, path.str() );

                bool consistent = true;
                try { newBot.prepare(); } catch( std::runtime_error& ) { consistent = false; }
                if ( consistent ) result.push_back( newBot );
            }
        }
    }
    for ( size_t i = 0; i < result.size(); ++i )
    {
        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/" << i << ".json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( result[i], path.str() );
    }

    path.str("");
    path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/parent.json";
    std::cout << "writing to " << path.str() << "\n";
    saveToFile( current, path.str() );

    return result;
}

// Make a vector of predecessors of target configuration 
// from map of predecessors
// Assume target is reachable from start in predecessors map
std::vector< RofiWorld > getPredecessors( const VisitedShapes& visited,
    std::unordered_map< VisitedId, VisitedId >& predecessor, 
    VisitedId target, VisitedId start )
{
    assert( predecessor.contains( start ) );
    assert( predecessor.contains( target ) );

    std::vector<RofiWorld> result;
    VisitedId current = target;
    
    while ( current != start ) {

        result.push_back( visited[current] );
        // current must always have a predecessor or given map is not correct
        assert( predecessor.contains( current ) );
        current = predecessor.find( current )->second;
    }

    result.push_back( visited[start] );
    std::reverse( result.begin(), result.end() );
    return result;
} 

// Assume target is reachable from start (same number of (only) **universal** modules)
// by using bound angles
std::vector<RofiWorld> bfsShapes(
    const RofiWorld& start, const RofiWorld& target,
    float step, unsigned int bound/*, BFSReporter& reporter*/ )
{
    VisitedShapes visited;
    
    std::unordered_map< VisitedId, VisitedId > predecessor;
    std::unordered_map< VisitedId, int > distance;

    VisitedId startId = visited.insert( start );
    // reporter.onUpdateSeen( visited );

    // Starting configuration has itself as predecessor
    // and distance of 0
    predecessor.insert( { startId, startId } );
    // reporter.onUpdatePredecessors( predecessor );
    distance.insert( { startId, 0 } );
    // reporter.onUpdateDistance( distance );

    // start is isomorphic to target
    if ( equalConfig( start, target ) ) 
        return getPredecessors( visited, predecessor, startId, startId );

    std::queue< VisitedId > bfsQueue;

    bfsQueue.push( startId );
    // reporter.onUpdateQueue( bfsQueue );

    while ( !bfsQueue.empty() ) 
    {
        VisitedId current = bfsQueue.front();
        // reporter.onUpdateCurrent( *current );
        bfsQueue.pop();
        // reporter.onUpdateQueue( bfsQueue );

        std::stringstream path;
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/start.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( start, path.str() );

        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/visitedStart.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( visited[startId], path.str() );

        assert( equalConfig( start, visited[startId] ) );
        assert( start.isPrepared() );
        std::vector< RofiWorld > descendants = getDescendants( visited[current], step, bound );
        assert( start.isPrepared() );

        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/start.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( start, path.str() );

        path.str("");
        path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/visitedStart.json";
        std::cout << "writing to " << path.str() << "\n";
        saveToFile( visited[startId], path.str() );

        assert( equalConfig( start, visited[startId] ) );

        int currchild = 0;
        for ( const RofiWorld& child : descendants ) {
            // if ( currchild == 4 || currchild == 5 )
            // {
            //     path.str("");
            //     path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/child" << currchild << ".json";
            //     std::cout << "writing to " << path.str() << "\n";
            //     saveToFile( child, path.str() );
            //     path.str("");
            //     path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/start.json";
            //     std::cout << "writing to " << path.str() << "\n";
            //     saveToFile( start, path.str() );
            //     path.str("");
            //     path << "/home/jarom/RoFI/softwareComponents/isoreconfig/bfsDesc/visitedStart.json";
            //     std::cout << "writing to " << path.str() << "\n";
            //     saveToFile( visited[startId], path.str() );
            //     assert( equalConfig( start, visited[startId] ) );
            //     assert( !equalConfig( child, start ) );
            //     assert( !equalConfig( child, visited[startId] ) );
            //     assert( !equalConfig( child, visited[0] ) );
            // } 
            ++currchild;
            if ( visited.contains( child ) ) continue;

            VisitedId childId = visited.insert( child );
            // reporter.onUpdateSeen( visitedChild );
            predecessor.insert({ childId, current });
            // reporter.onUpdatePredecessors(predecessor);
            assert( distance.find( current ) != distance.end() );
            distance.insert({ childId, distance.find( current )->second + 1 });
            // reporter.onUpdateDistance(distance);

            if ( equalConfig( child, target ) ) 
            {
                std::vector< RofiWorld > output = getPredecessors( visited, predecessor, childId, startId );
                // reporter.onBuildPredecessors( output );
                return output;
            }

            bfsQueue.push( childId );
            // reporter.onUpdateQueue( bfsQueue );
        }
    }
    std::vector< RofiWorld > output = {};
    // reporter.onBuildPredecessors(output);
    return output;
}

} // namespace rofi::isoreconfig
