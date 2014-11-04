package pomdp;

import pomdp.algorithms.ValueIteration;
import pomdp.algorithms.pointbased.ClusterValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.Logger;

public class POMDPSolver {

	public static void main(String[] args) {
		String sPath = "Models/";// 得到model路径
		String sModelName = "hallway";// model名
		String sMethodName = "PBVI";//方法名
		Logger.getInstance().setOutput(true);//允许输出
		Logger.getInstance().setSilent(false);//允许输出到控制台
		try {
			String sOutputDir = "logs/POMDPSolver";// 输出路径
			String sFileName = sModelName + "_" + sMethodName + ".txt";// 输出文件名
			Logger.getInstance().setOutputStream(sOutputDir, sFileName);
		} catch (Exception e) {
			System.err.println(e);
		}

		POMDP pomdp = null;
		double dTargetADR = 100.0;// 目标平均折扣回报值，控制结束条件
		try {
			pomdp = new POMDP();
			pomdp.load(sPath + sModelName + ".POMDP");// 载入pomdp模型
			
			//输出最大回报值和最小回报值
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
