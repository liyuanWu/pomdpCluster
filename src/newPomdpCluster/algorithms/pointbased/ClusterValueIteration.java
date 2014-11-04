package newPomdpCluster.algorithms.pointbased;



import java.io.File;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;

import newPomdpCluster.algorithms.ValueIteration;
import newPomdpCluster.environments.Model;
import newPomdpCluster.environments.POMDP;
import newPomdpCluster.utilities.AlphaVector;
import newPomdpCluster.utilities.BeliefState;
import newPomdpCluster.utilities.BeliefStateVector;
import newPomdpCluster.utilities.Pair;

public class ClusterValueIteration extends ValueIteration{
	protected Iterator<BeliefState> m_itCurrentIterationPoints;
    protected boolean m_bSingleValueFunction = true;
	 protected boolean m_bRandomizedActions;
	protected double m_dFilteredADR = 0.0;
	private Model model = new Model();
	ArrayList<MachineState> fsc = new ArrayList<MachineState>();
	 
	public ClusterValueIteration( POMDP pomdp ){
		super(pomdp);
		
		m_itCurrentIterationPoints = null;
		m_bRandomizedActions = true;
	}

	public ClusterValueIteration( POMDP pomdp, boolean bRandomizedActionExpansion ){
		super(pomdp);
		
		m_itCurrentIterationPoints = null;
		m_bRandomizedActions = bRandomizedActionExpansion;
	}
	


	public void expand(BeliefStateVector<BeliefState> pointset)
	{
		BeliefState bs = pointset.firstElement(); 
		
		while(pointset.size() < 5000)//�ռ�5000����
		{
			int iAction = m_rndGenerator.nextInt( m_pPOMDP.getActionCount() );
			int iObservation = m_rndGenerator.nextInt(m_pPOMDP.getObservationCount());
			double dOb = bs.probabilityOGivenA( iAction, iObservation );
			
			if(dOb>0.0)
			{
				//����b��a��oʱ�����b
				BeliefState bsNext = bs.nextBeliefState(iAction, iObservation);
				if(bsNext != null)
				{
					pointset.add(bs, bsNext);
					bs = bsNext;
				}
			}
		}
	}
	 public static double vecMultiply(double[] v1, double[] v2) {
	        double result = 0;
	        for (int i = 0; i < v1.length; i++) {
	            result += (v1[i] * v2[i]);

	        }
	        return result;
	    }

	  public void getPointsGrouped(ArrayList<MachineState> fsc, BeliefStateVector<BeliefState> pointset) {
	        for (BeliefState bs : pointset) {

	            int idx = -1;
	            double value = -Double.MAX_VALUE;
	            for (int k = 0; k < fsc.size(); k++) {
	                if (vecMultiply(bs.getStatePro(), fsc.get(k).vec) > value) {
	                    value = vecMultiply(bs.getStatePro(), fsc.get(k).vec);
	                    idx = k;
	                }
	            }

	            fsc.get(idx).pointGroup.add(bs.getStatePro());

	        }
	    }
	  
	  private void deleteMSwithNoPoint(ArrayList<MachineState> tempfsc) {
	        for (int i = tempfsc.size() - 1; i >= 0; i--) {
	            if (tempfsc.get(i).pointGroup.size() == 0) {
	                tempfsc.remove(i);
	            }
	        }
	    }
	  
	  private void completeLink(ArrayList<MachineState> tempfsc) {
	        for (int i = 0; i < tempfsc.size(); i++) {
	            double[] tp = tempfsc.get(i).pointGroup.get(0);
	            int action = tempfsc.get(i).action;
	            for (int k = 0; k < model.observations; k++) {
	                double[] nextPoint;
//	                
	                nextPoint = model.nextPointForHallway(tp, action, k);
	                if (new Double(nextPoint[0]).isNaN()) continue;


	                int outwardIdx = getMaxValueMachineStateIDX(tempfsc, nextPoint);
	                tempfsc.get(i).outwardLink.put(k, outwardIdx);

	            }
	        }
	    }
	  
	  private void setSamplePoints(ArrayList<MachineState> fsc) {
	        for (MachineState aFsc : fsc) {
	            LinkedList<double[]> tempSP;
	            tempSP = aFsc.samplepoints;

	            //��ά����+ֵ�����С��

	            for (int k = 0; k < model.states + 2; k++) {
	                tempSP.add(aFsc.pointGroup.get(0));
	            }


	            //�Ƚ���άֵ���ĵ������㼯��������������Сֵ�ĵ�
	            for (int k = 0; k < aFsc.pointGroup.size(); k++) {
	                for (int m = 0; m < model.states; m++) {
	                    if (aFsc.pointGroup.get(k)[m] > tempSP.get(m)[m]) {
	                        tempSP.set(m, aFsc.pointGroup.get(k));
	                    }
	                }
	                if (vecMultiply(aFsc.vec, aFsc.pointGroup.get(k))
	                        > vecMultiply(aFsc.vec, tempSP.get(model.states))) {
	                    tempSP.set(model.states, aFsc.pointGroup.get(k));
	                }
	                if (vecMultiply(aFsc.vec, aFsc.pointGroup.get(k))
	                        < vecMultiply(aFsc.vec, tempSP.get(model.states + 1))) {
	                    tempSP.set(model.states + 1, aFsc.pointGroup.get(k));
	                }

	            }


	        }
	    }
	
	public void initFSC(String sPath, BeliefStateVector<BeliefState> pointset)
	{
		 model.initModelFromFileForHallWay(new File(sPath));
	        //for spped up test.
	        model.initInerArray();
//	        model.initModelFromFile(modelfile);
	        ArrayList<MachineState> tempfsc = new ArrayList<MachineState>();
	        for (int i = 0; i < model.reward.size(); i++) {
	            MachineState tempMS = new MachineState();
	            tempMS.action = i;
	            tempMS.vec = model.reward.get(i);
	            tempfsc.add(tempMS);
	        }
	        getPointsGrouped(tempfsc, pointset);
	        deleteMSwithNoPoint(tempfsc);

	        completeLink(tempfsc);
	        setSamplePoints(tempfsc);
	        fsc = tempfsc;
	}
	
	public void clusterIteration(String sPath) {
		//maxRunningTime��numEvaluationsû���õ�  Ӧ��ɾ��
		
	BeliefStateVector<BeliefState> pointset = new BeliefStateVector<BeliefState>();
	pointset.add(null, m_pPOMDP.getBeliefStateFactory().getInitialBeliefState() );
	expand(pointset);//����5000����
	
	System.out.println("the size of pointset is " + pointset.size());
	
	
	}
	
	public void improveValueFunction(BeliefStateVector<BeliefState> vBeliefState)
	{
		ArrayList<ArrayList<BeliefState>> treeLevel = vBeliefState.getTreeLevelInfo();
		
		int levelSize = treeLevel.size();//�õ�����
		for(int i = levelSize - 1; i >= 0; --i)
		{
			ArrayList<BeliefState> level = treeLevel.get(i);//����õ�ÿһ�������㼯��
			
			m_itCurrentIterationPoints = level.iterator();
			while(m_itCurrentIterationPoints.hasNext())//����ÿһ��������
			{
				BeliefState bsCurrent = m_itCurrentIterationPoints.next();
				AlphaVector avCurrentMax = m_vValueFunction.getMaxAlpha( bsCurrent );//�õ���������
				AlphaVector avBackup = backup( bsCurrent );//���������
				
				double dBackupValue = avBackup.dotProduct( bsCurrent );
				double dValue = avCurrentMax.dotProduct( bsCurrent );
				double dDelta = dBackupValue - dValue;
				
				//���������Ż������µĦ�
				if(dDelta >= 0)
					m_vValueFunction.addPrunePointwiseDominated( avBackup );
			}
		}
	}

	protected boolean checkADRConvergence( POMDP pomdp, double dTargetADR, Pair<Double,Double> pComputedADRs ){
		double dSimulatedADR = 0.0;
		boolean bConverged = false;
		
		pComputedADRs.setFirst( new Double( 0.0 ) );
		pComputedADRs.setSecond( new Double( 0.0 ) );
		
		if( pomdp != null && g_cTrials > 0 ){
			dSimulatedADR = pomdp.computeAverageDiscountedReward( g_cTrials, g_cStepsPerTrial, this );
			
			if( m_dFilteredADR == 0.0 ){
				m_dFilteredADR = dSimulatedADR;
			}
			else{
				m_dFilteredADR = ( m_dFilteredADR + dSimulatedADR ) / 2;
				if( m_dFilteredADR >= dTargetADR )
					bConverged = true;
			}
			
			if( pComputedADRs != null ){
				pComputedADRs.setFirst( new Double( dSimulatedADR ) );
				pComputedADRs.setSecond( new Double( m_dFilteredADR ) );
			}						
		}
		return bConverged || m_bTerminate;
	}
	
	public int getAction(BeliefState bsCurrent) {
		return m_vValueFunction.getBestAction(bsCurrent);
	}
	
}

class MachineState implements Serializable {
    int action;
    double[] vec;
    HashMap outwardLink = new HashMap();
    LinkedList<double[]> pointGroup = new LinkedList<double[]>();
    LinkedList<double[]> samplepoints = new LinkedList<double[]>();

    @Override
    public boolean equals(Object obj) {
        return this.vec.equals(((MachineState) obj).vec);
    }
}
