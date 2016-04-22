package me.unkei.unkei;

import android.app.Activity;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Bundle;
import android.view.Gravity;
import android.view.TextureView;
import android.view.View;
import android.widget.FrameLayout;

import java.io.IOException;

public class ScanSender extends Activity implements TextureView.SurfaceTextureListener {

        private Camera mCamera;
        private TextureView mTextureView;

        @Override
        protected void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);

            mTextureView = new TextureView(this);
            mTextureView.setSurfaceTextureListener(this);

            setContentView(mTextureView);
        }

        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
            mCamera = Camera.open();

            Camera.Size previewSize = mCamera.getParameters().getPreviewSize();
            mTextureView.setLayoutParams(new FrameLayout.LayoutParams(
                    previewSize.width, previewSize.height, Gravity.CENTER));

            try {
                mCamera.setPreviewTexture(surface);
            } catch (IOException t) {
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
}
