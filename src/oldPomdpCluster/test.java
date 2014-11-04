package oldPomdpCluster;

import java.io.*;
import java.util.*;

/**
 * Created with IntelliJ IDEA.
 * User: hanbing
 * Date: 14-2-9
 * Time: 下午8:53
 * To change this template use File | Settings | File Templates.
 */
public class test {
    public static void main(String[] args) throws IOException {
//        int totalCount=0;
//        double e=0.37;
//        int minp=20;
//        LinkedList<double[]> mylist =oldPomdpCluster.UtilClass.readActAndObv("actAndObv.out");
//        oldPomdpCluster.Model model = new oldPomdpCluster.Model();
//        oldPomdpCluster.PomdpSolver ps = new oldPomdpCluster.PomdpSolver();
//        ps.readPoints(new File("pointset"));
//
////        for(double x=0;x<1;x+=0.01){
//
//            totalCount=0;
//            Dbscan.newapplyDbscan("pointset",e,minp);
//            List<List<Point>> resultList=Dbscan.resultList;
//
//            for(int i=resultList.size()-1;i>=0;i--){
//                if(resultList.get(i).size()==0){
//                    resultList.remove(i);
//                }else{
//                    totalCount+=resultList.get(i).size();
//                }
//
//            }
//            System.out.println(e+" "+totalCount+" "+resultList.size());
//
//        //resultList.clear();
//
////        }
//
//
//
//
//        for(int m=0;m<resultList.size();m++){
//            for(Point p:resultList.get(m)){
//                int idx=p.idx;
//                int nextGroup=-1;
//                if(idx==0) continue;
//                double[] aAo = mylist.get(idx);
//                for(int i=0;i<resultList.size();i++){
//                    for(int k=0;k<resultList.get(i).size();k++){
//                        if(resultList.get(i).get(k).idx==idx+1){
//                            nextGroup=i;
//                        }
//                    }
//                }
//                if(nextGroup==-1) continue;
//                System.out.println(aAo[0]+" "+aAo[1]+" "+nextGroup);
//
//
//            }
//            System.out.println("-------------------------------------------------------------------");
//        }


        HashSet hs = new HashSet();
        hs.add(new Integer(1));
        System.out.println(hs.contains(1));








        System.out.println(123);




    }
}
