@routing  @guidance @mode-change
Feature: Notification on turn onto mode change

    Background:
        Given the profile "car"
        Given a grid size of 20 meters

    Scenario: Turn onto a Ferry
        Given the node map
            | h |   |   |   |   |   |
            |   |   |   |   |   | f |
            | b | c |   |   | d | e |
            |   |   |   |   |   |   |
            | a |   |   |   |   | g |

        And the ways
            | nodes | highway | route | name  |
            | ab    | primary |       | bingo |
            | bc    | primary |       | dingo |
            | cde   |         | ferry | ferry |
            | eg    | primary |       | wingo |

        When I route I should get
            | waypoints | route                   | turns                                                                          |
            | a,g       | bingo,ferry,wingo,wingo | depart,new name straight,new name right,arrive                                 |

    Scenario: Turn onto a Ferry
        Given the node map
            |   |   |   |   |   |   | g |
            |   | a |   |   |   |   | e |
            |   |   | c |   |   | d |   |
            |   |   |   |   |   |   | f |
            |   | b |   |   |   |   |   |

        And the ways
            | nodes | highway | route | name  | oneway |
            | ca    | primary |       | bingo | yes    |
            | bc    | primary |       | dingo | yes    |
            | cde   |         | ferry | ferry | no     |
            | cdf   |         | ferry | ferry | no     |
            | ge    | primary |       | wingo | no     |

        When I route I should get
            | waypoints | route                   | turns                                                                          |
            | g,a       | wingo,ferry,ferry,bingo,bingo | depart,new name right,continue right,new name right,arrive               |

    Scenario: Straight onto a Ferry
        Given the node map
            |   |   |   |   |   |   | g |   |
            |   |   |   |   |   |   |   |   |
            | a | b | c |   |   | d |   | i |
            |   |   |   |   |   |   | f |   |
            |   |   |   |   |   |   |   |   |

        And the ways
            | nodes | highway | route | name    |
            | ab    | primary |       | yonkers |
            | bc    | primary |       | dingo   |
            | ca    | primary |       | bingo   |
            | cd    |         | ferry | ferry   |
            | fdc   |         | ferry | ferry   |
            | gi    | primary |       | wingo   |
            | fi    | primary |       | dolly   |

        When I route I should get
            | waypoints | route                            | turns                                                                          |
            | a,i       | yonkers,ferry,ferry,dolly,dolly | depart,new name right,continue right,new name right,arrive               |
