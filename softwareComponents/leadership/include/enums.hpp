namespace rofi::leadership {
    enum MessageType {
            ELECTION_MESSAGE,  // Carries basic election messages 
            LEADER_MESSAGE,    // Announces leadership
            INITIATE_MESSAGE,  // For the initiation of the algorithm
            CONNECT_MESSAGE,
            FOLLOWER_CHANGE_MESSAGE,
        };

    enum RelayType {
        SEND_TO_NEIGHBORS, // All but parent
        SEND_TO_PARENT,    // Only parent
        SEND_TO_ALL,       // Everyone
        DO_NOT_SEND,       // Stops message sending
    };

    enum ElectionStatus {
        UNDECIDED,
        LEADER,
        FOLLOWER,
        CHANGED_FOLLOWERS,
    };
}