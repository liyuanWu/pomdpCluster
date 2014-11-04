package oldPomdpCluster.dbscan;

/**
 * Created with IntelliJ IDEA.
 * User: hanbing
 * Date: 14-2-17
 * Time: 下午5:25
 * To change this template use File | Settings | File Templates.
 */
public class Point {
    public double[] dim;
    private boolean isKey;
    private boolean isClassed;

    public int idx =-1;

    public boolean isKey() {
        return isKey;
    }

    public void setKey(boolean isKey) {
        this.isKey = isKey;
        this.isClassed = true;
    }

    public boolean isClassed() {
        return isClassed;
    }

    public void setClassed(boolean isClassed) {
        this.isClassed = isClassed;
    }



    public Point(String str) {
        String[] p = str.split(" ");

        dim= new double[p.length];
        for(int i=0;i<p.length;i++){
            dim[i]=Double.valueOf(p[i]);
        }
    }
    public Point(String str,int idx) {
        String[] p = str.split(" ");

        dim= new double[p.length];
        for(int i=0;i<p.length;i++){
            dim[i]=Double.valueOf(p[i]);
        }
        this.idx=idx;
    }

    public String print() {
        StringBuilder sb = new StringBuilder();
        for(int i=0;i<dim.length-1;i++){
            sb.append(dim[i]+" ");

        }
        sb.append(dim[dim.length-1]);


        return sb.toString();
    }
}

