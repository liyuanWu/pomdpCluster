package oldPomdpCluster;

import oldPomdpCluster.dbscan.Dbscan;
import oldPomdpCluster.dbscan.Point;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.Serializable;
import java.math.BigDecimal;
import java.util.*;

/**
 * Created with IntelliJ IDEA.
 * User: hanbing
 * Date: 14-1-31
 * Time: 下午4:14
 * To change this template use File | Settings | File Templates.
 */
public class PomdpSolver {
    Model model = new Model();
    ArrayList<MachineState> fsc = new ArrayList<MachineState>();
    LinkedList<double[]> pointList = new LinkedList<double[]>();
    //for clusterPI

    List<List<Point>> clusterPointList;
    boolean firstIter = true;
    List<BackupPoint> backupPointList = new LinkedList<BackupPoint>();
    double ave=Double.NaN;
    List<Integer> unclusteredPointList= new LinkedList<Integer>();





    //for pbpi

    LinkedList<double[]> bSet = new LinkedList<double[]>();
    LinkedList<double[]> bNextSet = new LinkedList<double[]>();
    double bSetOldADR=Double.NaN;
    boolean canStartIter=false;


    public void initFSC(File modelfile, File pointList) throws Exception {

        model.initModelFromFileForHallWay(modelfile);

        //for spped up oldPomdpCluster.test.
        model.initInerArray();

//        model.initModelFromFile(modelfile);
        ArrayList<MachineState> tempfsc = new ArrayList<MachineState>();
        for (int i = 0; i < model.reward.size(); i++) {
            MachineState tempMS = new MachineState();
            tempMS.action = i;
            tempMS.vec = model.reward.get(i);
            tempfsc.add(tempMS);
        }
        readPoints(pointList);
        getPointsGrouped(tempfsc);
        deleteMSwithNoPoint(tempfsc);

        completeLink(tempfsc);
        setSamplePoints(tempfsc);
        fsc = tempfsc;
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
//                try {
//                    nextPoint = model.nextPoint(tp, action, k);
//                } catch (oldPomdpCluster.NextPointNotExistException e) {
//                    e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
//                    continue;
//                }
                nextPoint = model.nextPointForHallway(tp, action, k);
                if (new Double(nextPoint[0]).isNaN()) continue;


                int outwardIdx = getMaxValueMachineStateIDX(tempfsc, nextPoint);
                tempfsc.get(i).outwardLink.put(k, outwardIdx);

            }
        }


    }

    public static int getMaxValueMachineStateIDX(ArrayList<MachineState> fsc, double[] point) {
        int idx = -1;
        double value = -Double.MAX_VALUE;
        for (int i = 0; i < fsc.size(); i++) {
            if (vecMultiply(fsc.get(i).vec, point) > value) {
                value = vecMultiply(fsc.get(i).vec, point);
                idx = i;
            }
        }
        return idx;


    }


    public boolean convergenceForPBPI(ArrayList<MachineState> rear,double convergenceArg){
        if(Double.isNaN(bSetOldADR)){
            double sum=0;
            for(double[] point: bSet){
                sum+=vecMultiply(point,rear.get(getMaxValueMachineStateIDX(rear,point)).vec);
            }
            bSetOldADR=sum/bSet.size();
            return false;
        }

        double bSetNewADR=0;
        for(double[] point:bSet){
            bSetNewADR+=vecMultiply(point,rear.get(getMaxValueMachineStateIDX(rear,point)).vec);
        }
        bSetNewADR=bSetNewADR/bSet.size();

        System.out.println(bSetNewADR);


        double ratio=Math.abs((bSetNewADR-bSetOldADR)/bSetOldADR)+1;
        //System.out.println(ratio);
        bSetOldADR=bSetNewADR;


        return ratio<=1+convergenceArg;


    }

    //使用类似pbpi07收敛条件
    public boolean convergence(ArrayList<MachineState> fore, ArrayList<MachineState> rear, double convergenceArg) {
        double pvar = pointValueAve(rear);
        double pvaf = pointValueAve(fore);
//        double ratio = pointValueAve(rear) / pointValueAve(fore);
        double ratio = Math.abs((pvar - pvaf) / pvaf) + 1;
        System.out.println(pvar);
//        System.out.println(ratio);
        return ratio <= 1 + convergenceArg;
    }


    public double getAve(ArrayList<MachineState> fsc){
        double sum = 0;
        for(BackupPoint bp: backupPointList){
            sum+=PomdpSolver.vecMultiply(bp.point,fsc.get(PomdpSolver.getMaxValueMachineStateIDX(fsc,bp.point)).vec);

        }
        return sum/backupPointList.size();
    }
    public boolean convergenceWithB(ArrayList<MachineState> fore, ArrayList<MachineState> rear, double convergenceArg) {


        double pvar = getAve(rear);
        double pvaf = getAve(fore);
//        double ratio = pointValueAve(rear) / pointValueAve(fore);
        double ratio = Math.abs((pvar - pvaf) / pvaf) + 1;
        System.out.println(pvar);
        return ratio <= 1 + convergenceArg;
    }

    private double pointValueAve(ArrayList<MachineState> fsc) {
        BigDecimal sumVal = new BigDecimal(0);
        for (MachineState aFsc : fsc) {
            for (int k = 0; k < aFsc.pointGroup.size(); k++) {
                sumVal = sumVal.add(new BigDecimal(vecMultiply(aFsc.vec, aFsc.pointGroup.get(k))));
            }
        }

        return sumVal.divide(new BigDecimal(pointList.size()), 10, BigDecimal.ROUND_HALF_UP).doubleValue();


    }

    public ArrayList<MachineState> findOptimalPolicyWithPBPI(double gama, double[] startPoint) {
        ArrayList<MachineState> fore = fsc;
//        ArrayList<oldPomdpCluster.MachineState> rear = PI(fore,gama);
        ArrayList<MachineState> rear = PBPI(fore, gama, startPoint);

        while (!convergenceForPBPI(rear,0.01)){//||bSet.size()<model.states) {
//        while (bSet.size()<60) {
//            fore = rear;
//            rear = PI(rear,gama);
            rear = PBPI(rear, gama, startPoint);
            //System.out.println(oldPomdpCluster.UtilClass.ADR(pointList,rear));

        }
        completeLink(rear);
        return rear;
    }

    private ArrayList<MachineState> PBPI(ArrayList<MachineState> fore, double gama, double[] startPoint) {
        if (this.bSet.size() == 0) {
            bSet.add(startPoint);
            bNextSet = getAllNextPoints(bSet,model);//只有初始信念点  取得所有后继点



            BackupPoint tbp = new BackupPoint();
            tbp.point=bSet.get(0);
            backupPointList.add(tbp);//需要被更新的点集   现在只有初始信念点

            for(BackupPoint bp:backupPointList ){
                for(int action=0;action<model.actions;action++){//遍历每一个动作
                    Map<Integer,double[]> nextPoints= new HashMap<Integer,double[]>();
                    bp.actionList.add(nextPoints);
                    for(int obv=0;obv<model.observations;obv++){//遍历每一个状态 得到相应的信念点
                        double[] nextPointWithActAndObv = model.nextPointForHallway(bp.point,action,obv);
                        bp.actionList.get(action).put(obv,nextPointWithActAndObv);
                    }
                }

            }



        }

        double[] newPoint = findMostDistPoint(bSet, bNextSet);//距离最远的点
        bSet.add(newPoint);
        LinkedList<double[]> temp = new LinkedList<double[]>();
        temp.add(newPoint);
        bNextSet.addAll(getAllNextPoints(temp,model));//bNextSet不会越来越大吗

        BackupPoint bp = new BackupPoint();//保存信念点  以及该信念点所有的后继点
        bp.point=newPoint;
        for(int action=0;action<model.actions;action++){
            Map<Integer,double[]> nextPoints= new HashMap<Integer,double[]>();
            bp.actionList.add(nextPoints);
            for(int obv=0;obv<model.observations;obv++){
                double[] nextPointWithActAndObv = model.nextPointForHallway(bp.point,action,obv);
                bp.actionList.get(action).put(obv,nextPointWithActAndObv);
            }
        }
        backupPointList.add(bp);



        ArrayList<MachineState> newMSList = new ArrayList<MachineState>();
        for (int i = 0; i < backupPointList.size(); i++) {

            ArrayList al = newbackup(fore, backupPointList.get(i), gama);//返回向量  第一个是index，第二个是向量
            MachineState tempMS = new MachineState();
            tempMS.vec = (double[]) al.get(1);
            tempMS.action = (Integer) al.get(0);
            newMSList.add(tempMS);//信念点更新之后的MachineState集
        }


        for (MachineState ms : fore) {
            MachineState tempMS = new MachineState();
            tempMS.vec = ms.vec;
            tempMS.action = ms.action;
            newMSList.add(tempMS);//把fore的MachineState加到newMSList中
        }


        HashSet tempSet = new LinkedHashSet<MachineState>(newMSList);
        newMSList.clear();
        newMSList.addAll(tempSet);
        getPointsGroupedForPBPI(newMSList);
        deleteMSwithNoPoint(newMSList);
        //completeLink(newMSList);
        //setSamplePoints(newMSList);
        return newMSList;


    }




    public static LinkedList<double[]> getAllNextPoints(LinkedList<double[]> bSet,Model model) {
        int actions = model.actions;
        int obvs = model.observations;
        LinkedList<double[]> ret = new LinkedList<double[]>();
        for (double[] point : bSet) {
            for (int action = 0; action < actions; action++) {
                for (int obv = 0; obv < obvs; obv++) {
                    double[] next = model.nextPointForHallway(point, action, obv);
//                    if (!new Double(next[0]).isNaN()) {
                        ret.add(next);
//                    }
                }
            }
        }
        return ret;

    }

    public static double[] findMostDistPoint(LinkedList<double[]> bSet, LinkedList<double[]> bNextSet) {
        int idx = -1;
        double len = -Double.MAX_VALUE;
        for (int pointIdx = 0; pointIdx < bNextSet.size(); pointIdx++) {
            if(new Double(bNextSet.get(pointIdx)[0]).isNaN()) continue;
            double tempLen = getLen(bNextSet.get(pointIdx), bSet);
            if (tempLen > len) {
                len = tempLen;
                idx = pointIdx;
            }

        }
        return bNextSet.get(idx);

    }

    public static double getLen(double[] point, LinkedList<double[]> bSet) {
        double len = 0;
        for (int i = 0; i < bSet.size(); i++) {
            double tempLen = 0;
            for (int s = 0; s < bSet.get(i).length; s++) {
                tempLen += (point[s] - bSet.get(i)[s]) * (point[s] - bSet.get(i)[s]);
            }
            tempLen = Math.sqrt(tempLen);
            len += tempLen;
        }
        return len;

    }

    public ArrayList<MachineState> findOptimalPolicy(double gama) {
        ArrayList<MachineState> fore = this.fsc;

//        ArrayList<oldPomdpCluster.MachineState> rear = PI(fore,gama);
        ArrayList<MachineState> rear = fore;//clusterPI(fore, gama);
        //System.out.println("pointListSize:"+clusterPointList.size());

        while (!convergence(fore, rear, 0.01)||this.firstIter) {
            fore = rear;
//            rear = PI(rear,gama);

            rear = clusterPI(rear, gama);
        }
        completeLink(rear);
        return rear;
    }

    public ArrayList<MachineState> clusterPI(ArrayList<MachineState> fsc, double gama) {
        if (this.firstIter == true) {//第一次
            try {
                Dbscan.applyDbscan("pointset");
                clusterPointList = Dbscan.resultList;//点已经成簇
                Random rand = new Random();


                //find unclustered points.
                List<Integer> clusteredPoints = new LinkedList<Integer>();
                for(int i=0;i<clusterPointList.size();i++){
                    if(clusterPointList.get(i).size()>0){
                        for(Point p: clusterPointList.get(i)){
                            clusteredPoints.add(p.idx);//找到成簇的点
                        }
                    }
                }
                HashSet hs = new HashSet(clusteredPoints);

                for(int i=0;i<pointList.size();i++){
                    if(!hs.contains(i)){
                        unclusteredPointList.add(i);//找到没有成簇的点，即散点
                    }
                }



                //add cluster sample points to point set.
                for(int i=0;i<clusterPointList.size();i++){
                    if(clusterPointList.get(i).size()>0){
                        BackupPoint bp = new BackupPoint();
                        int idx = rand.nextInt(clusterPointList.get(i).size());
                        bp.point=clusterPointList.get(i).get(idx).dim;
                        backupPointList.add(bp);
                        clusterPointList.get(i).remove(idx);//从簇中random一个点，然后删除这个点
                    }//不需要把unclusterdpoint里随机选一个点？
                }
                System.out.println(backupPointList.size());

                for(BackupPoint bp:backupPointList ){
                    for(int action=0;action<model.actions;action++){
                        Map<Integer,double[]> nextPoints= new HashMap<Integer,double[]>();
                        bp.actionList.add(nextPoints);
                        for(int obv=0;obv<model.observations;obv++){
                            double[] nextPointWithActAndObv = model.nextPointForHallway(bp.point,action,obv);
                            bp.actionList.get(action).put(obv,nextPointWithActAndObv);//计算出所有后继点
                        }
                    }

                }

                //remove elemt in clusterPointList whose size is 0.
                for(int i=clusterPointList.size()-1;i>=0;i--){
                    if(clusterPointList.get(i).size()==0){
                        clusterPointList.remove(i);
                    }
                }








            } catch (IOException e) {
                e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
                System.exit(1);
            }

        }


        //add points to point set.
        if(firstIter==false){
            double chooseUncluster=0.2;//1-1.0*unclusteredPointList.size()/pointList.size();
            Random rand = new Random();
            double choose = rand.nextDouble();
            double[] nextPoint;
            if(choose<chooseUncluster){//从unclustered里面随机取一个点
                int unclusterIdx = rand.nextInt(unclusteredPointList.size());
                nextPoint=pointList.get(unclusteredPointList.get(unclusterIdx));
                unclusteredPointList.remove(unclusterIdx);
            }else{//从clustered里面随机取一个点
                int sum=0;
                for(int i=0;i<clusterPointList.size();i++){
                    sum+=clusterPointList.get(i).size();
                }
                int idx =-1;
                int randNum = rand.nextInt(sum);
                sum=0;
                for(int i=0;i<clusterPointList.size();i++){
                    if(randNum>sum&&randNum<sum+clusterPointList.get(i).size()){
                        idx=i;
                        break;
                    }
                    sum+=clusterPointList.get(i).size();

                }
                nextPoint=clusterPointList.get(idx).get(0).dim;
                if(clusterPointList.get(idx).size()==1){
                    clusterPointList.remove(0);
                }else{
                    clusterPointList.get(idx).remove(0);
                }
            }

            double[] newPoint = nextPoint;
//            bSet.add(newPoint);
//            LinkedList<double[]> temp = new LinkedList<double[]>();
//            temp.add(newPoint);
//            bNextSet.addAll(getAllNextPoints(temp,model));

            BackupPoint bp = new BackupPoint();
            bp.point=newPoint;
            for(int action=0;action<model.actions;action++){
                Map<Integer,double[]> nextPoints= new HashMap<Integer,double[]>();
                bp.actionList.add(nextPoints);
                for(int obv=0;obv<model.observations;obv++){
                    double[] nextPointWithActAndObv = model.nextPointForHallway(bp.point,action,obv);
                    bp.actionList.get(action).put(obv,nextPointWithActAndObv);
                }
            }
            backupPointList.add(bp);



        }
        this.firstIter = false;

        //backup details.
        ArrayList<MachineState> newMSList = new ArrayList<MachineState>();

        for (int clusterIdx = 0; clusterIdx < backupPointList.size(); clusterIdx++) {
//            System.out.println("one point backup done.");

            Model.printPoint(backupPointList.get(clusterIdx).point);
            ArrayList al = newbackup(fsc, backupPointList.get(clusterIdx), gama);
            MachineState tempMS = new MachineState();//得到向量和动作
            tempMS.vec = (double[]) al.get(1);
            tempMS.action = (Integer) al.get(0);
            newMSList.add(tempMS);
        }


        for (MachineState ms : fsc) {//把fsc中的machinestate加到newMSList中
            MachineState tempMS = new MachineState();
            tempMS.vec = ms.vec;
            tempMS.action = ms.action;
            newMSList.add(tempMS);
        }


        HashSet tempSet = new LinkedHashSet<MachineState>(newMSList);
        newMSList.clear();
        newMSList.addAll(tempSet);

//        getPointsGrouped(newMSList);
        getPointsGroupedForClusterPI(newMSList);
        deleteMSwithNoPoint(newMSList);
        //completeLink(newMSList);
        //setSamplePoints(newMSList);
        return newMSList;


    }


    public ArrayList<MachineState> PI(ArrayList<MachineState> fsc, double gama) {
        ArrayList<MachineState> newMSList = new ArrayList<MachineState>();
        for (int i = 0; i < fsc.size(); i++) {
            for (int k = 0; k < fsc.get(i).samplepoints.size(); k++) {
                System.out.println("point:" + k);
                ArrayList al = backup(fsc, fsc.get(i).samplepoints.get(k), gama);
                MachineState tempMS = new MachineState();
                tempMS.vec = (double[]) al.get(1);
                tempMS.action = (Integer) al.get(0);
                newMSList.add(tempMS);
            }
        }


        for (MachineState ms : fsc) {
            MachineState tempMS = new MachineState();
            tempMS.vec = ms.vec;
            tempMS.action = ms.action;
            newMSList.add(tempMS);
        }


        HashSet tempSet = new LinkedHashSet<MachineState>(newMSList);
        newMSList.clear();
        newMSList.addAll(tempSet);
        getPointsGrouped(newMSList);
        deleteMSwithNoPoint(newMSList);
        //completeLink(newMSList);
        setSamplePoints(newMSList);
        return newMSList;


//        LinkedList<double[]> newVecList = new LinkedList<double[]>();
//        LinkedList<Integer> newVecListAction = new LinkedList<Integer>();
//        for (int i = 0; i < fsc.size(); i++) {
//            for (int k = 0; k < fsc.get(i).samplepoints.size(); k++) {
//
//                //此处有问题，backup后向量对应的动作呢？
//                ArrayList al  = backup(fsc, fsc.get(i).samplepoints.get(k), 0.3);
//                double[] tempVec=(double [])al.get(1);
//                newVecList.add(tempVec);
//                newVecListAction.add((Integer)al.get(0));
//            }
//        }
//        HashSet th =new HashSet<double[]>(newVecList);
//        newVecList.clear();
//        newVecList.addAll(th);
//
//        //把原fsc的向量放入newVecList中
//        for (oldPomdpCluster.MachineState aFsc : fsc) {
//            newVecList.add(aFsc.vec);
//        }
//
//
//        ArrayList<oldPomdpCluster.MachineState> newMSList;
//        newMSList = makeMSListFromVec(newVecList);
//        getPointsGrouped(newMSList);
//        deleteMSwithNoPoint(newMSList);
//        completeLink(newMSList);
//        setSamplePoints(newMSList);


//        return newMSList;
    }

    //返回的list中第一个元素是动作号，第二个元素是向量
    private ArrayList backup(ArrayList<MachineState> fsc, double[] point, double gama) {
        LinkedList<double[]> vecList = new LinkedList<double[]>();


        LinkedList<BackupActionVecAtPoint> bavapList = new LinkedList<BackupActionVecAtPoint>();


        for (int k = 0; k < model.actions; k++) {
            double[] temp = new double[model.states];
//            for (int i = 0; i < model.states; i++) {
//                System.out.println(i);
//                temp[i] = model.reward.get(k)[i] + gama * SumAZ(i, k, fsc, point);
//            }
            BackupActionVecAtPoint bavap = new BackupActionVecAtPoint(this, point, fsc, k, temp, gama, vecList);
            bavap.start();
            bavapList.add(bavap);
//            vecList.add(temp);
        }

        for (BackupActionVecAtPoint bavap : bavapList) {
            try {
                bavap.join();
            } catch (InterruptedException e) {
                e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
            }
        }


        return maxVecAtPoint(vecList, point);
    }


    private ArrayList newbackup(ArrayList<MachineState> fsc, BackupPoint bp, double gama) {
        LinkedList<double[]> vecList = new LinkedList<double[]>();


        LinkedList<NewBackupActionVecAtPoint> bavapList = new LinkedList<NewBackupActionVecAtPoint>();


        for (int k = 0; k < model.actions; k++) {
            double[] temp = new double[model.states];
//            for (int i = 0; i < model.states; i++) {
//                temp[i] = rsa(this,k,i) + gama * newSumAZ(i, k, fsc, bp);
//            }
//
//            vecList.add(temp);


            NewBackupActionVecAtPoint bavap = new NewBackupActionVecAtPoint(this, bp, fsc, k, temp, gama, vecList);
            bavap.start();
            bavapList.add(bavap);//开多个线程计算向量  更新vecList

        }

        for (NewBackupActionVecAtPoint bavap : bavapList) {
            try {
                bavap.join();
            } catch (InterruptedException e) {
                e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
            }
        }


        return maxVecAtPoint(vecList, bp.point);
    }

    public double SumAZ(int stateIdx, int action, ArrayList<MachineState> fsc, double[] point) {
        double result = 0;

        for (int transToStateIdx = 0; transToStateIdx < model.states; transToStateIdx++) {

            for (int observ = 0; observ < model.observations; observ++) {
                double[] nextPoint = model.nextPointForHallway(point, action, observ);
                if (new Double(nextPoint[0]).isNaN()) continue;

                result += model.transList.get(action)[stateIdx][transToStateIdx]
                        * model.obserList.get(action)[transToStateIdx][observ]
                        * fsc.get(getMaxValueMachineStateIDX(fsc, nextPoint)).vec[transToStateIdx];
            }
        }
        return result;
    }

    public double newSumAZ(int stateIdx, int action, ArrayList<MachineState> fsc, BackupPoint bp) {
        double result = 0;

        for (int transToStateIdx = 0; transToStateIdx < model.states; transToStateIdx++) {

            for (int observ = 0; observ < model.observations; observ++) {
                double[] nextPoint = bp.actionList.get(action).get(observ);//oldPomdpCluster.Model.nextPointForHallwayStatic(model, point, action, observ);
                if (new Double(nextPoint[0]).isNaN()) continue;

                result += model.transArray[action][stateIdx][transToStateIdx]
                        * model.obserArray[action][transToStateIdx][observ]
                        * fsc.get(getMaxValueMachineStateIDX(fsc, nextPoint)).vec[transToStateIdx];
            }
        }
        return result;
    }

    //返回arraylist，第一个元素是idx，第二个元素是向量
    private ArrayList maxVecAtPoint(LinkedList<double[]> vecList, double[] point) {
        ArrayList al = new ArrayList();
        double value = -Double.MAX_VALUE;
        int idx = -1;
        for (int i = 0; i < vecList.size(); i++) {
            if (vecMultiply(vecList.get(i), point) > value) {
                idx = i;
                value = vecMultiply(vecList.get(i), point);
            }
        }
        al.add(idx);
        al.add(vecList.get(idx));

        return al;


    }


    public static ArrayList<MachineState> makeMSListFromVec(List<double[]> vecList) {
        ArrayList<MachineState> temp = new ArrayList<MachineState>();
        for (double[] aVecList : vecList) {
            MachineState tempMS = new MachineState();
            tempMS.vec = aVecList;
            temp.add(tempMS);
        }
        return temp;
    }

    public void readPoints(File pointSet) throws FileNotFoundException {
        Scanner scanner = new Scanner(pointSet);
        while (scanner.hasNext()) {
            String[] ts = scanner.nextLine().split(" ");
            double[] tp = new double[ts.length];
            for (int i = 0; i < ts.length; i++) {
                tp[i] = Double.valueOf(ts[i]);
            }
            pointList.add(tp);
        }
    }

    public void getPointsGroupedForPBPI(ArrayList<MachineState> fsc) {
        for (double[] point:bSet) {

            int idx = -1;
            double value = -Double.MAX_VALUE;
            for (int k = 0; k < fsc.size(); k++) {
                if (vecMultiply(point, fsc.get(k).vec) > value) {
                    value = vecMultiply(point, fsc.get(k).vec);
                    idx = k;
                }
            }

            fsc.get(idx).pointGroup.add(point);


        }
    }
    public void getPointsGrouped(ArrayList<MachineState> fsc) {
        for (double[] aPointList : pointList) {

            int idx = -1;
            double value = -Double.MAX_VALUE;
            for (int k = 0; k < fsc.size(); k++) {
                if (vecMultiply(aPointList, fsc.get(k).vec) > value) {
                    value = vecMultiply(aPointList, fsc.get(k).vec);
                    idx = k;
                }
            }

            fsc.get(idx).pointGroup.add(aPointList);


        }
    }
    public void getPointsGroupedForClusterPI(ArrayList<MachineState> fsc) {
        for (BackupPoint aPointList : backupPointList) {

            int idx = -1;
            double value = -Double.MAX_VALUE;
            for (int k = 0; k < fsc.size(); k++) {
                if (vecMultiply(aPointList.point, fsc.get(k).vec) > value) {
                    value = vecMultiply(aPointList.point, fsc.get(k).vec);
                    idx = k;
                }
            }

            fsc.get(idx).pointGroup.add(aPointList.point);


        }
    }

    public static double vecMultiply(double[] v1, double[] v2) {
        double result = 0;
        for (int i = 0; i < v1.length; i++) {
            result += (v1[i] * v2[i]);

        }
        return result;
    }


    public static void main(String[] args) {

//        double[] firstPoint = new double[]{0.011419,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.0,0.0,0.0,0.0,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,
//                0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363};
//        double gama=0.95;
//        String modelFilePath = "hallway2.POMDP";




        double[] firstPoint = new double[]{0.017865, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857,
                0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.0, 0.0, 0.0, 0.0};
        double gama=0.95;
        String modelFilePath = "hallway.POMDP";
        System.out.println(firstPoint.length);


//        double[] firstPoint = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//        double gama = 0.95;
//        String modelFilePath = "tiger-grid.POMDP";



//        double[] firstPoint = new double[]{0.1,0.2,0.3,0.4};
//        double gama=1;
//        String modelFilePath = "e:\\cheng.D4-1.POMDP";


        try {
            Model.writePointSetToFile(firstPoint, modelFilePath);//输出结果到pointset文件中

        } catch (FileNotFoundException e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }

//
        PomdpSolver ps = new PomdpSolver();
        ArrayList<MachineState> result;


        try {
            ps.initFSC(new File(modelFilePath), new File("pointset"));//读取pointset文件，初始化FSC

            result = ps.findOptimalPolicy(gama);
//            result=ps.findOptimalPolicyWithPBPI(gama,firstPoint);

            UtilClass.saveFSC(result,"cluster-hw-1.out");
            System.out.println(123);


        } catch (Exception e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }


    }
    private double rsa(PomdpSolver ps,int action,int state) {
        double rsa=0;
        double[][][] ta=ps.model.transArray;
        double[][] ra=ps.model.rewardArray;
        for(int t=0;t<ps.model.states;t++){
            rsa+=ta[action][state][t]*ra[action][t];
        }
        return rsa;

    }

    private void setSamplePoints(ArrayList<MachineState> fsc) {
        for (MachineState aFsc : fsc) {
            LinkedList<double[]> tempSP;
            tempSP = aFsc.samplepoints;

            //各维最大点+值最大最小点

            for (int k = 0; k < model.states + 2; k++) {
                tempSP.add(aFsc.pointGroup.get(0));
            }


            //先将各维值最大的点放入样本点集，最后放入最大和最小值的点
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


class BackupActionVecAtPoint extends Thread {
    PomdpSolver ps;
    int action;
    ArrayList<MachineState> fsc;
    double[] point;
    double[] ret;
    double gama;
    LinkedList<double[]> vecList;


    BackupActionVecAtPoint(PomdpSolver ps, double[] point, ArrayList<MachineState> fsc, int action, double[] ret, double gama, LinkedList<double[]> vecList) {
        this.ps = ps;
        this.point = point;
        this.fsc = fsc;
        this.action = action;
        this.ret = ret;
        this.gama = gama;
        this.vecList = vecList;
    }

    @Override
    public void run() {
        for (int i = 0; i < ps.model.states; i++) {
            ret[i] = ps.model.reward.get(action)[i] + gama * ps.SumAZ(i, action, fsc, point);

        }
        vecList.add(ret);
    }
}
class NewBackupActionVecAtPoint extends Thread {
    PomdpSolver ps;
    int action;
    ArrayList<MachineState> fsc;
    BackupPoint bp;
    double[] ret;
    double gama;
    LinkedList<double[]> vecList;


    NewBackupActionVecAtPoint(PomdpSolver ps, BackupPoint bp, ArrayList<MachineState> fsc, int action, double[] ret, double gama, LinkedList<double[]> vecList) {
        this.ps = ps;
        this.bp=bp;
        this.fsc = fsc;
        this.action = action;
        this.ret = ret;
        this.gama = gama;
        this.vecList = vecList;
    }

    @Override
    public void run() {
        for (int i = 0; i < ps.model.states; i++) {
            ret[i] = ps.model.reward.get(action)[i] + gama * ps.newSumAZ(i, action, fsc, bp);
//            ret[i]=rsa(ps,action,i)
//                    + gama * ps.newSumAZ(i, action, fsc, bp);
        }
        vecList.add(ret);
    }

    private double rsa(PomdpSolver ps,int action,int state) {
        double rsa=0;
        double[][][] ta=ps.model.transArray;
        double[][] ra=ps.model.rewardArray;
        for(int t=0;t<ps.model.states;t++){
            rsa+=ta[action][state][t]*ra[action][t];
        }
        return rsa;

    }
}
class BackupPoint{
    //k,v=obv,nextpoint
    double[] point;
    //Map<Integer,double[]>
    ArrayList<Map<Integer,double[]>> actionList= new ArrayList<Map<Integer,double[]>>();
    //		actionList下标为动作  map的Integer为观察  double[]对应信念点
}

//for (int i = 0; i < model.states; i++) {
//        System.out.println(i);
//temp[i] = model.reward.get(k)[i] + gama * SumAZ(i, k, fsc, point);
//}


