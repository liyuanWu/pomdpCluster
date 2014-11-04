package newPomdpCluster;

import newPomdpCluster.algorithms.PolicyStrategy;
import newPomdpCluster.utilities.BeliefState;
import newPomdpCluster.utilities.RandomGenerator;

/**
 * @author shanigu
 *
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */

public class RandomWalkPolicy extends PolicyStrategy {
	private int m_cActions;
	protected RandomGenerator m_rndGenerator;
	
	public RandomWalkPolicy( int cActions ){
		m_cActions = cActions;
		m_rndGenerator = new RandomGenerator( "RandomWalk" );
	}
	public int getAction( BeliefState bsCurrent ){
		return m_rndGenerator.nextInt( m_cActions );
	}
}
