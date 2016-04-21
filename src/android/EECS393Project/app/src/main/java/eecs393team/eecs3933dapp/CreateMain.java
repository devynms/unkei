package eecs393team.eecs3933dapp;

import android.app.Activity;
import android.content.Intent;
import android.net.Uri;
import android.os.Environment;
import android.provider.MediaStore;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.Toast;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.TimeUnit;
import android.hardware.Camera;

public class CreateMain extends Activity {

    static final int REQUEST_VIDEO_CAPTURE = 200;
    public static final int MEDIA_TYPE_VIDEO = 2;

    private static final int CAPTURE_VIDEO_ACTIVITY_REQUEST_CODE = 200;


    private String ip;
    private ServerConnection myConnection;
    private Intent nextState;
    private Uri fileUri;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_create_main);
        setServer("172.19.14.127");

        connectToServer();
    }

    protected void setServer(String ip){
        myConnection = new ServerConnection(ip);
        this.ip = ip;
    }

    public ServerConnection getServer(){
        return myConnection;
    }

    protected void connectToServer(){
        //spawn loading icon
        //connect to server
        boolean success = false;
        ServerConnection new_server = (ServerConnection) new ServerConnection(ip).execute(); //myConnection.connectToServer();
        try {
            success = new_server.get(20, TimeUnit.SECONDS);

        } catch(Exception e){

        }
        Button next_button = (Button) findViewById(R.id.button);
        //if true
        if (success) {
            //spawn check
            ImageView connecting_graphic = (ImageView) findViewById(R.id.imageView);
            connecting_graphic.setImageResource(R.drawable.check_mark);
            //spawn start_video button
            nextState = new Intent(android.provider.MediaStore.ACTION_VIDEO_CAPTURE);

            nextState.putExtra("ServerConnection", myConnection);
            next_button.setText(R.string.start_video);
        }
        //else
        else {
            // spawn failure
            ImageView connecting_graphic = (ImageView) findViewById(R.id.imageView);
            connecting_graphic.setImageResource(R.drawable.red_x);
            // spawn back button
            nextState = new Intent(this, MainActivity.class);
            next_button.setText(R.string.back);
        }
        next_button.setClickable(true);
    }

    public void nextState(View view){
        if (nextState.resolveActivity(getPackageManager()) != null) {
            fileUri = getOutputMediaFileUri(MEDIA_TYPE_VIDEO);  // create a file to save the video
            nextState.putExtra(MediaStore.EXTRA_OUTPUT, fileUri);  // set the image file name

            nextState.putExtra(MediaStore.EXTRA_VIDEO_QUALITY, 1);
            startActivityForResult(nextState, REQUEST_VIDEO_CAPTURE);
        }
        else {
            startActivity(nextState);
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (requestCode == CAPTURE_VIDEO_ACTIVITY_REQUEST_CODE) {
            if (resultCode == RESULT_OK) {
                // Video captured and saved to fileUri specified in the Intent
                Toast.makeText(this, "Video saved to:\n" +
                        data.getData(), Toast.LENGTH_LONG).show();
            } else if (resultCode == RESULT_CANCELED) {
                // User cancelled the video capture
            } else {
                // Video capture failed, advise user
            }
        }
    }

    /** Create a file Uri for saving an image or video */
    private static Uri getOutputMediaFileUri(int type){
        return Uri.fromFile(getOutputMediaFile(type));
    }

    /** Create a File for saving an image or video */
    private static File getOutputMediaFile(int type){
        // To be safe, you should check that the SDCard is mounted
        // using Environment.getExternalStorageState() before doing this.

        File mediaStorageDir = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_PICTURES), "MyCameraApp");
        // This location works best if you want the created images to be shared
        // between applications and persist after your app has been uninstalled.

        // Create the storage directory if it does not exist
        if (! mediaStorageDir.exists()){
            if (! mediaStorageDir.mkdirs()){
                Log.d("MyCameraApp", "failed to create directory");
                return null;
            }
        }

        // Create a media file name
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        File mediaFile;
        if(type == MEDIA_TYPE_VIDEO) {
            mediaFile = new File(mediaStorageDir.getPath() + File.separator +
                    "VID_"+ timeStamp + ".mp4");
        } else {
            return null;
        }

        return mediaFile;
    }

}
