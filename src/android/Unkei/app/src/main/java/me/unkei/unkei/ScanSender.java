package me.unkei.unkei;

import android.app.Activity;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.util.Log;
import android.view.Gravity;
import android.view.TextureView;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.LinearLayout;

import java.io.IOException;
import java.util.List;

public class ScanSender extends Activity implements TextureView.SurfaceTextureListener {

    private Camera mCamera;
    private TextureView mTextureView;

    private MediaRecorder mMediaRecorder;

    private ImageButton recordButton;
    private Boolean isRecording;

        @Override
        protected void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);

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

            root.addView(recordButton);
            isRecording = false;

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

        }

    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, the Camera does all the work for us
    }

    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
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
}
