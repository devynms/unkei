package me.unkei.unkei;

import android.app.Activity;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.media.CamcorderProfile;
import android.media.MediaRecorder;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Gravity;
import android.view.TextureView;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.LinearLayout;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;

public class ScanSender extends Activity implements TextureView.SurfaceTextureListener {

    private Camera mCamera;
    private TextureView mTextureView;

    private MediaRecorder mMediaRecorder;

    private ImageButton recordButton;
    private Boolean isRecording;
    private Boolean isReady = false;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        isRecording = false;

        mTextureView = new TextureView(this); //(TextureView) findViewById(R.id.surface_view);
        mTextureView.setSurfaceTextureListener(this);
        setContentView(R.layout.activity_scan_sender);
        FrameLayout root = (FrameLayout) findViewById(R.id.root);
        root.addView(mTextureView);
        recordButton = new ImageButton(this);
        recordButton.setImageResource(R.drawable.start_recording);
        recordButton.setMinimumWidth(200);

        FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(FrameLayout.LayoutParams.WRAP_CONTENT, FrameLayout.LayoutParams.FILL_PARENT);
        params.gravity = Gravity.RIGHT;
        recordButton.setLayoutParams(params);

        recordButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    if (isRecording) {
                        recordButton.setImageResource(R.drawable.start_recording);
                        mMediaRecorder.stop();
                        mMediaRecorder.reset();
                        isRecording = false;

                        try {
                            prepRecorder(mTextureView.getSurfaceTexture());
                        } catch(IOException e){
                            Log.e("Recording","can't prep the recorder" + e.getMessage());
                        }
                    }
                    else {
                        isRecording = true;
                        recordButton.setImageResource(R.drawable.stop_recoding);
                        mMediaRecorder.start();
                    }
                }
            });

        root.addView(recordButton);

        setContentView(root);
    }

    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
/*
        Camera.Parameters parameters = mCamera.getParameters();
        Camera.Size previewSize = mTextureView.getSize //getBestPreviewSize();
        mTextureView.setLayoutParams(new FrameLayout.LayoutParams(
                previewSize.width, previewSize.height, Gravity.CENTER));
*/
        try {
            mCamera.setPreviewTexture(surface);

        } catch (IOException t) {
            Log.e("Scan", "Unable to set preview texture" + t.getMessage());
        }

        mCamera.startPreview();

        try {
            if(!isReady) {
                mCamera.unlock();
                prepRecorder(surface);
            }
        } catch (IOException e) {
            Log.e("Scan", "Unable to prepare Recorder" + e.getMessage());
        }
    }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, the Camera does all the work for us
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        releaseMediaRecorder();
        mCamera.release();
        return true;
    }

    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // Update your view here!
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

    private void prepRecorder(SurfaceTexture surfaceTexture) throws IOException {
        if(mCamera == null){
            mCamera = Camera.open();
            mCamera.unlock();
        }

        if(mMediaRecorder == null)
            mMediaRecorder = new MediaRecorder();

        mMediaRecorder.setCamera(mCamera);

        CamcorderProfile profile = CamcorderProfile.get(CamcorderProfile.QUALITY_HIGH);

        mMediaRecorder.setVideoSource(MediaRecorder.VideoSource.CAMERA);
        mMediaRecorder.setAudioSource(MediaRecorder.AudioSource.CAMCORDER);
        //mMediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.DEFAULT);
        mMediaRecorder.setProfile(profile);


        File output = getOutputMediaDir();//getOutputMediaFile();
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        String path = output.getAbsolutePath() + "/Scan_" + timeStamp + ".mp4";
        mMediaRecorder.setOutputFile(path);
        mMediaRecorder.setMaxDuration(180000); // 3 minutes maximum
        //mMediaRecorder.setVideoFrameRate(30);

        //mMediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.);

        try {
            mMediaRecorder.prepare();
        } catch (IllegalStateException e){
            Log.e("Recorder", "Couldn't prepare Recorder");
        }
        isReady = true;
    }

    private static Uri getOutputMediaFileUri(){
        return Uri.fromFile(getOutputMediaFile());
    }

    private static File getOutputMediaDir() {

        File mediaStorageDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES), "UnkeiScans");

        // Create the storage directory if it does not exist
        if (!mediaStorageDir.exists()) {
            if (!mediaStorageDir.mkdirs()) {
                Log.d("Unkei", "failed to create directory");
                return null;
            }
        }
        return mediaStorageDir;
    }


    private static File getOutputMediaFile(){

        File mediaStorageDir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES), "UnkeiScans");

        // Create the storage directory if it does not exist
        if (! mediaStorageDir.exists()){
            if (! mediaStorageDir.mkdirs()){
                Log.d("Unkei", "failed to create directory");
                return null;
            }
        }

        // Create a media file name
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        File mediaFile;

        mediaFile = new File(mediaStorageDir.getPath() + File.separator + "SCAN_"+ timeStamp + ".mp4");

        return mediaFile;
    }

    private void releaseMediaRecorder(){
        if(mMediaRecorder != null){
            mMediaRecorder.reset();
            mMediaRecorder.release();
            mMediaRecorder = null;
        }
    }

}
