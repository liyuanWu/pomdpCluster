package newPomdpCluster.utilities.distance;

import newPomdpCluster.utilities.BeliefState;

public interface DistanceMetric 
{
    double distance(BeliefState bs1, BeliefState bs2);
}
