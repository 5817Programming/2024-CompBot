public class AutoExecuter{

    private Thread mThread = null;

    private AutoBase mAuto = null;


    public void setAuto(AutoBase auto){
        this.mAuto = auto;
    }

    public void getAuto(){
        return this.mAuto;
    }

    public void start(){
        if(mThread == null){
            mThread = new Thread(new CrashTrackingRunnable(){
                @Override
                public void runCrashTracked(){
                    if(auto != null)
                        auto.start();   
                }
            });
 
        }
   }

    public void stop(){
        if(auto != null){
            auto.stop();
        }
        mThread = null;
    }
}    public void setAuto(AutoBase auto){
