package newPomdpCluster;

import newPomdpCluster.algorithms.ValueIteration;
import newPomdpCluster.algorithms.pointbased.ClusterValueIteration;
import newPomdpCluster.environments.POMDP;
import newPomdpCluster.utilities.Logger;

public class POMDPSolver {

	public static void main(String[] args) {
		String sPath = "Models/";// �õ�model·��
		String sModelName = "hallway";// model��
		String sMethodName = "PBVI";//������
		Logger.getInstance().setOutput(true);//�������
		Logger.getInstance().setSilent(false);//�������������̨
		try {
			String sOutputDir = "logs/POMDPSolver";// ���·��
			String sFileName = sModelName + "_" + sMethodName + ".txt";// ����ļ���
			Logger.getInstance().setOutputStream(sOutputDir, sFileName);
		} catch (Exception e) {
			System.err.println(e);
		}

		POMDP pomdp = null;
		double dTargetADR = 100.0;// Ŀ��ƽ���ۿۻر�ֵ�����ƽ�������
		try {
			pomdp = new POMDP();
			pomdp.load(sPath + sModelName + ".POMDP");// ����pomdpģ��
			
			//������ر�ֵ����С�ر�ֵ
    	    //Logger.getInstance().logln("max is " + pomdp.getMaxR() + " min is " + pomdp.getMinR());
		} catch (Exception e) {
			Logger.getInstance().logln(e);
			e.printStackTrace();
			System.exit(0);
		}
		
		ValueIteration iteration = new ClusterValueIteration(pomdp);
		iteration.clusterIteration(sPath);
		
	}
}
