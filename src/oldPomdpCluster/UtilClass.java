package oldPomdpCluster;

import java.io.*;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Random;

/**
 * Created by hanbing on 14-2-28.
 */
public class UtilClass {
    public static LinkedList<double[]> readActAndObv(String filePath){
        try {
            FileInputStream fis = new FileInputStream(filePath);
            ObjectInputStream ois = new ObjectInputStream(fis);
            return (LinkedList<double[]>)ois.readObject();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
        return null;
    }
    public static void saveFSC(ArrayList<MachineState> fsc,String outputFileName){
        try {
            FileOutputStream fos = new FileOutputStream(outputFileName);
            ObjectOutputStream oos = new ObjectOutputStream(fos);
            oos.writeObject(fsc);
        } catch (FileNotFoundException e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        } catch (IOException e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }

    }
    public static ArrayList<MachineState> readFSC(String filePath){
        try {
            FileInputStream fis = new FileInputStream(filePath);
            ObjectInputStream ois = new ObjectInputStream(fis);
            return (ArrayList<MachineState>)ois.readObject();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
        return null;
    }
    public static double sumForB(double[] point){
        double sum=0;
        for(int i=0;i<point.length;i++){
            sum+=point[i];
        }
        return sum;
    }

    public static double rewardAve(int steps, ArrayList<MachineState> fsc,double[] startPoint,String modelFilePath){
        Model model = new Model();
        model.initModelFromFileForHallWay(new File(modelFilePath));
        double sumReward=0;
        double[] pointNow = startPoint;
        double gama =1;

//        int count=0;



        for(int i=0;i<steps;i++){
            MachineState ms =fsc.get(PomdpSolver.getMaxValueMachineStateIDX(fsc,pointNow));
            int action = ms.action;

            int obv = randObv(pointNow,fsc,model);
//            System.out.println(sumForB(pointNow));
//            oldPomdpCluster.Model.printPoint(pointNow);


//            int  idx=0;
//            double val=0;
//            for(int m=0;m<model.states;m++){
//                if(pointNow[m]>val){
//                    idx=m;
//                    val=pointNow[m];
//                }
//
//            }

//            if(idx==8||idx==9||idx==10||idx==11){
//                count++;
//            }
//            if(idx==0||idx==1||idx==2||idx==3||idx==16||idx==17||idx==18||idx==19){
//                count--;
////                sumReward+=oldPomdpCluster.PomdpSolver.vecMultiply(pointNow,model.reward.get(action));
//                sumReward+=oldPomdpCluster.PomdpSolver.vecMultiply(pointNow,ms.vec);
////                System.out.println(count);
//                return sumReward;
//            }


//            sumReward+=gama*oldPomdpCluster.PomdpSolver.vecMultiply(pointNow,ms.vec);
            sumReward+=gama*PomdpSolver.vecMultiply(pointNow,model.reward.get(action));
            gama=gama*0.95;

            pointNow=model.nextPointForHallway(pointNow,action,obv);

        }

//        System.out.println(count);


        return sumReward;
    }
    public static double nTimesTryAve(int times,int steps, ArrayList<MachineState> fsc,double[] startPoint,String modelFilePath){
        double sum = 0;
        for(int t = 0;t<times;t++){
            sum+=rewardAve(steps,fsc,startPoint,modelFilePath);
//            double tempsum=0;
//            double[] sp = randStartPoint(fsc.get(0).vec.length);
//
//            for(int i=0;i<10;i++){
//                tempsum+=rewardAve(steps,fsc,sp,modelFilePath);
//                tempsum=tempsum/10;
//            }
//            sum+=tempsum;
        }
        return sum/times;

    }
    public static double[] randStartPoint(int statesNum){
        Random rand = new Random();
        double[] point = new double[statesNum];
        double sum=1;
        for(int i=0;i<statesNum-1;i++){
            point[i]=sum*rand.nextDouble();
            sum-=point[i];


        }
        point[statesNum-1]=sum;
        return point;
    }

//    public static double ADR(List<double[]> pointList,ArrayList<oldPomdpCluster.MachineState> fsc){
//        double sum=0;
//        for(double[] point: pointList){
//            sum+=oldPomdpCluster.PomdpSolver.vecMultiply(point,fsc.get(oldPomdpCluster.PomdpSolver.getMaxValueMachineStateIDX(fsc,point)).vec);
//        }
//        return sum/pointList.size();
//    }


    public static int randObv(double[] point,ArrayList<MachineState> fsc,Model model){
        double sum=0;
        double[] pointNow = point;
        List<double[]> nextPoints= new LinkedList<double[]>();
        MachineState ms = fsc.get(PomdpSolver.getMaxValueMachineStateIDX(fsc,pointNow));
        for(int i=0;i<model.observations;i++){
            double[] next = model.nextPointForHallway(pointNow,ms.action,i);
            nextPoints.add(next);
        }
        double[] p = new double[model.observations];

        for(int i=0;i<model.observations;i++){
            if(new Double(nextPoints.get(i)[0]).isNaN()){
                p[i]=0;
            }else{
                double ps=0;
                for(int k=0;k<model.states;k++){
                    ps+=model.obserList.get(ms.action)[k][i]*nextPoints.get(i)[k];
                }
                p[i]=ps;
            }
        }

        for(double tp:p){
            sum+=tp;
        }
        Random rand = new Random();

        double randret = rand.nextDouble()*sum;
        double tempSum=0;
//        System.out.print("p:");
//        oldPomdpCluster.Model.printPoint(p);
        for(int i=0;i<p.length;i++){

            if(randret>=tempSum&&randret<=tempSum+p[i])
                return i;
            else{
                tempSum+=p[i];
            }
        }




        return -1;



    }
//    public static int simpleRandObv(double[] point,ArrayList<oldPomdpCluster.MachineState> fsc,oldPomdpCluster.Model model){
//        Random rand = new Random();
//        int sIdx =-1;
//        double val=0;
//        for(int i=0;i<model.states;i++){
//            if(point[i]>val){
//                val=point[i];
//                sIdx=i;
//            }
//        }
//        double sum=0;
//        oldPomdpCluster.MachineState ms = fsc.get(oldPomdpCluster.PomdpSolver.getMaxValueMachineStateIDX(fsc,point));
//        for(int i=0;i<model.observations;i++){
//            if()
//        }
//
//    }



    public static void main(String[] args) {
        //tiger-grid
        double[] firstPoint = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //hallway
//        firstPoint = new double[]{0.017865, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857,
//                0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.0, 0.0, 0.0, 0.0};
//        //hallway2
//        firstPoint = new double[]{0.011419,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.0,0.0,0.0,0.0,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,
//                0.011363,0.011363,0.011363,0.011363,0.011363,0.011363,0.011363};

        ArrayList<MachineState> result= readFSC("pbpi-tg-1.out");
      //  ArrayList<oldPomdpCluster.MachineState> result= readFSC("cluster-tg-1.out");
        double ave =nTimesTryAve(10, 200, result, firstPoint, "tiger-grid.POMDP");
        System.out.println(ave);





//        oldPomdpCluster.PomdpSolver ps = new oldPomdpCluster.PomdpSolver();
//        try {
//            ps.initFSC(new File("tiger-grid.POMDP"),new File("pointset-tiger-grid"));
//            ArrayList<oldPomdpCluster.MachineState> result= readFSC("result.out");
//            double sum = 0;
//            double temp=0;
//            for(double[] point:ps.pointList){
//                sum=0;
//                for(int i=0;i<point.length;i++){
//                    sum+=point[i];
//
//                }
//                if(sum>1.1){
//                    System.out.println("warn");
//                }
//
//            }
//
//
//        } catch (Exception e) {
//            e.printStackTrace();
//        }


    }


}
