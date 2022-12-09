#pragma once

#include "lwip++.hpp"
#include <networking/interface.hpp>
#include <networking/routingTable.hpp>

#include <vector>

namespace rofi::net {

/**
 * \brief Virtual class defining the interface for NetworkManager's protocols.
*/
class Protocol {
    std::function< RoutingTable::Records () > _routingTableFun;

public:
    /**
     * \brief Enum for communicating the route updates.
    */
    enum class Route { ADD     /// A new route to be added
                     , RM      /// A known route is no longer valid
                    };

    /**
     * \brief Enum for communicating the interface state updates.
    */
    enum class ConfigAction { ADD_IP     /// Interface shoud get a new IP
                            , REMOVE_IP  /// Interface should lose an IP
                            , SHUT_DOWN  /// Interface should be shut down
                            , SET_UP     /// Interface should be set up
                            , RESPOND    /// A response on obtained update is needed
                            };

    using RoutingTableFun = std::function< RoutingTable::Records () >;
    using ConfigChange    = std::tuple< Interface::Name, Ip6Addr, uint8_t >;

    virtual ~Protocol() = default;

    /**
     * \brief Process the incomming message on given interface.
     * 
     * \return true if if a change of state is required.
     * 
    */
    virtual bool onMessage( const std::string& interfaceName, hal::PBuf packet ) = 0;

    /**
     * \brief This function is called when an event (Connected, Disconnected) on connector arises.
     *
     * \return true if any the event caused any changes.
     *
     * @param interface on which event originated
     * @param connected true if a connector was connected, false if it was disconnected
     */
    virtual bool onInterfaceEvent( const Interface& /* interface */, bool /* connected */ ) { return false; }

    /** 
     * This function is called after processing updates from onMessage (if any) and provides
     * to the protocol the ability of sending a message via given interface.
     * 
     * \return bool indicating if any local change is required to take place.
    */ 
    virtual bool afterMessage( const Interface& i, std::function< void ( PBuf&& ) > f, void* args ) = 0;

    /**
     * \brief Function that returns true if there are any pending routing updates.
     */
    virtual bool hasRouteUpdates() const { return false; }

    /**
     * \brief Function that outputs all routing updates possibly created in `onMessage`.
     * 
     * \return A vector of pairs with the type of change (e.g., a new route) and its RoutingTable::Record to be added.
     * 
     * This function is called in case `onMessage` and `hasRouteUpdates` return `true`. The default
     * implementation returns an empty vector, so it is not necessary to override this for non-routing protocols.
     * 
    */
    virtual std::vector< std::pair< Route, RoutingTable::Record > > getRouteUpdates() const { return {}; };

    /**
     * \brief Function that returns true if there are any pending configuration updates.
     */
    virtual bool hasConfigUpdates() const { return false; }

    /**
     * \brief Function that outputs all interface/state related updates possibly created in `onMessage`
     * 
     * \return A vector of pairs with the type of change (e.g., a new IP) and its `ConfigChange` with
     * all necessary information.
     * 
     * This function is called in case `onMessage` and `hasConfigUpdates` return `true`. The default
     * imlementation return an empty vector, so it is not necessary to override this for routing-only protocols.
    */
    virtual std::vector< std::pair< ConfigAction, ConfigChange > > getConfigUpdates() const { return {}; }

    /**
     * \brief Clear any (cached) updates.
     * 
     * This function should clear all the updates within the protocol. By calling this function, the NetworkManager
     * signalizes that all the updates (both – routing and interface related) were gathered already and are no longer
     * needed.
    */
    // TODO: Think about renaming
    virtual void clearUpdates() = 0;

    /**
     * \brief Indicate a new address on given interface.
     * 
     * \return true if any change was caused by this change.
    */
    virtual bool addAddressOn( const Interface&, const Ip6Addr&, uint8_t /* mask */ ) { return false; }

    /**
     * \brief Indicate a removal of an address on given interface.
     * 
     * \return true if any change was caused by this change.
    */
    virtual bool rmAddressOn( const Interface&, const Ip6Addr&, uint8_t /* mask */ ) { return false; }


    /**
     * \brief Add the given interface into the protocol.
     * 
     * \return true if the interface was succesfully added.
     * 
    */
    virtual bool addInterface( const Interface& interface ) = 0;

    /**
     * \brief Remove the given interface into the protocol.
     * 
     * \return true if the interface was succesfully removed.
     * 
    */
    virtual bool removeInterface( const Interface& interface ) = 0;

    /**
     * \brief Given an interface, decide if it is managed by the protocol.
     * 
     * \return true if the interface is managed by the protocol.
     * 
     * Namely, it should hold that after adding an interface via `addInterface`
     * which returns true, then `manages` should return true for such interface.
    */
    virtual bool manages( const Interface& interface ) const = 0;

    /**
     * \brief This function can set the callback for the routing table.
     * 
     * The callback itself should be a function that returns the current state of the table
     * for given protocol – records learned through it. 
    */
    void setRoutingTableCB( RoutingTableFun f ) {
        if ( f )
            _routingTableFun = f;
    }

    /**
     * \brief Function that outputs all the records from routing table related to the protocol.
     * 
     * Internally, it uses the callback set in `setRoutingTableCB` function. If any such function
     * is provided, then this function returns an empty collection. 
    */
    RoutingTable::Records routingTableCB() const {
        if ( _routingTableFun )
            return { _routingTableFun() };

        return {};
    }

    /**
     * \brief Returns a multicast IP on which the protocol listener will be running.
     * 
     * This should be unique for each protocol. 
    */
    virtual Ip6Addr address() const = 0;

    /**
     * \brief Returns a name of the protocol.
     * 
     * This should be unique for each protocol. 
    */
    virtual std::string name() const = 0;

    /**
     * \brief Returns a string representing some basic information about
     * the protocol (e.g., its name and listenner address). 
    */
    virtual std::string info() const {
        std::stringstream ss;
        ss << name() << "; address: " << address();
        return ss.str();
    }
};

} // namespace rofi::net
