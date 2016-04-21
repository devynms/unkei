package me.unkei.unkei;

import android.app.Activity;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.media.CamcorderProfile;
import android.media.MediaRecorder;
import android.net.Uri;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Gravity;
import android.view.TextureView;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageButton;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

/** written by josh, inspired in part by the google sample on media recording from a camera*/
public class ScanActivity extends Activity implements TextureView.SurfaceTextureListener {

    private Camera mCamera;
    private TextureView mPreview;
    private MediaRecorder mMediaRecorder;

    private ImageButton recordButton;
    private Boolean isRecording;
    private Boolean isReady = false;

    @Override
    protected void onCreate(Bundle savedInstanceState){
        super.onCreate(savedInstanceState);

        mPreview = (TextureView) findViewById(R.id.surface_view);
        recordButton = (ImageButton) findViewById(R.id.button_record);
        isRecording = false;

        setContentView(R.layout.activity_scan);
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();

        Camera.Size previewSize = mCamera.getParameters().getPreviewSize();
        mPreview.setLayoutParams(new FrameLayout.LayoutParams(
                previewSize.width, previewSize.height, Gravity.CENTER));

        try {
            mCamera.setPreviewTexture(surface);
        } catch (IOException t) {
            Log.e("Scan", "Can't set Preview Texture" + t.getMessage());
        }

        mCamera.startPreview();

    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {

    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        onSurfaceTextureAvailable(surface,mPreview.getWidth(),mPreview.getHeight());
    }

    public void onRecordClick(View view){
        if(!isRecording){
            //start recording
            new MediaRecorderPrep().execute(null,null,null);
        }
        else{
            //stop recording and save video
            mMediaRecorder.stop();
            releaseMediaRecorder();
            //mCamera.lock();

            recordButton.setImageResource(R.drawable.start_recording);
            isRecording = false;
            releaseCamera();
        }
    }

    private void releaseMediaRecorder(){
        if(mMediaRecorder != null){
            mMediaRecorder.reset();
            mMediaRecorder.release();
            mMediaRecorder = null;
            //mCamera.lock();
        }
    }

    private void releaseCamera(){
        if(mCamera != null){
            mCamera.release();
            mCamera = null;
        }
    }

    private boolean prepRecorder(){
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        Camera.Size previewSize = parameters.getPreviewSize();

        CamcorderProfile profile = CamcorderProfile.get(CamcorderProfile.QUALITY_HIGH);
        profile.videoFrameWidth = previewSize.width;
        profile.videoFrameHeight = previewSize.height;


        /*Camera.Size optimalSize = getBestPreviewSize(parameters.getSupportedPreviewSizes(), mPreview.getWidth(), mPreview.getHeight());

        CamcorderProfile profile = CamcorderProfile.get(CamcorderProfile.QUALITY_HIGH);
        profile.videoFrameWidth = optimalSize.width;
        profile.videoFrameHeight = optimalSize.height;

        parameters.setPreviewSize(profile.videoFrameWidth, profile.videoFrameHeight);
        mCamera.setParameters(parameters);

        try {
            mCamera.setPreviewTexture(mPreview.getSurfaceTexture());
        } catch (IOException e){
            Log.e("Scan", "Surface Texture unavailable" + e.getMessage());
            return false;
        }
        */
        mMediaRecorder = new MediaRecorder();
        //mCamera.unlock();
        mMediaRecorder.setCamera(mCamera);
        //we don't need to record audio but apparently we need to define the source
        mMediaRecorder.setAudioSource(MediaRecorder.AudioSource.CAMCORDER);
        mMediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);
        mMediaRecorder.setProfile(profile);
        mMediaRecorder.setOutputFile(getOutputMediaFile().getPath());

        try {
            mMediaRecorder.prepare();
        } catch (IllegalStateException e){
            Log.d("Scan", "preparing media recorder" + e.getMessage());
            releaseMediaRecorder();
            return false;
        } catch (IOException e){
            Log.d("Scan", "preparing media recorder" + e.getMessage());
            releaseMediaRecorder();
            return false;
        }
        return true;
    }

    /** Create a file Uri for saving an image or video */
    private static Uri getOutputMediaFileUri(){
        return Uri.fromFile(getOutputMediaFile());
    }

    /** Create a File for saving an image or video */
    private static File getOutputMediaFile(){
        File mediaStorageDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES), "Unkei Scans");
        // This location works best if you want the created images to be shared
        // between applications and persist after your app has been uninstalled.

        // Create the storage directory if it does not exist
        if (! mediaStorageDir.exists()){
            if (! mediaStorageDir.mkdirs()){
                Log.d("Unkei Storage", "failed to create directory");
                return null;
            }
        }

        // Create a media file name
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        File mediaFile;
        mediaFile = new File(mediaStorageDir.getPath() + File.separator + "SCAN_"+ timeStamp + ".mp4");

        return mediaFile;
    }

    public static Camera.Size getBestPreviewSize(List<Camera.Size> sizes, int previewWidth, int previewHeight){
        final double ASPECT_TOLERANCE = 0.1;
        double targetRatio = (double) previewWidth/previewHeight;
        if(sizes == null)
            return null;

        double minErr = Double.MAX_VALUE;
        Camera.Size optimal = null;
        Camera.Size best = null;
        double ratio = 0;
        for (Camera.Size size : sizes){
            ratio = (double) size.width/size.height;
            if(Math.abs(size.height - previewHeight) < minErr){
                minErr = Math.abs(size.height - previewHeight);
                optimal = size;
                if(Math.abs(ratio - targetRatio) < ASPECT_TOLERANCE)
                    best = optimal;
            }
        }
        if(best != null)
            return best;
        return optimal;
    }

    /*@Override
    protected void onPause(){
        super.onPause();
        //free hardware and software
        releaseMediaRecorder();
        releaseCamera();
    }*/

    class MediaRecorderPrep extends AsyncTask<Void,Void,Boolean> {

        @Override
        protected Boolean doInBackground(Void... params) {
            if(isReady){
                mMediaRecorder.start();
                isRecording = true;
            } else{
                releaseMediaRecorder();
                return false;
            }
            return  true;
        }

        @Override
        protected  void onPostExecute(Boolean result){
            if(!prepRecorder()){
                ScanActivity.this.finish();
            }
            recordButton.setImageResource(R.drawable.stop_recoding);
        }
    }
}