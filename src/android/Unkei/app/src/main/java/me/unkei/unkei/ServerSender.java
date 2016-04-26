package me.unkei.unkei;

import android.app.ProgressDialog;
import android.content.Intent;
import android.os.AsyncTask;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.concurrent.TimeUnit;

public class ServerSender extends AppCompatActivity {

    private static String ip;
    private Socket server;
    private int port = 8080;
    public File outputFile;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_server_sender);
        Bundle extras = getIntent().getExtras();
        if (extras != null) {
            outputFile = (File) extras.get("output_path");
        }
        ip = MainActivity.serverIp;
        SenderTask new_server = (SenderTask) new SenderTask().execute(outputFile.getPath());
        boolean success = false;
        try {
            success = new_server.get(20, TimeUnit.SECONDS);

        } catch (Exception e) {

        }
        if (success) {
            //spawn check
            Log.d("ScanActivity", "Starting to send");
            OutputStream outputStream = new_server.getServerOutput();
            InputStream inputStream = new_server.getServerInput();
            FileInputStream in = null;
            try {
                Log.d("ScanActivity", "Creating file" + outputFile.getPath());
                in = new FileInputStream(outputFile.getPath());
                // Write to the stream:
                byte[] buffer = new byte[1024]; // 1KB buffer size
                int length = 0;
                Log.d("ScanActivity", "Begining send");
                while ((length = in.read(buffer, 0, buffer.length)) != -1) {
                    Log.d("ScanActivity", "sending" + length);
                    outputStream.write(buffer, 0, length);
                }
                Log.d("ScanActivity", "Sent");
                outputStream.flush();
            } catch (Exception e) {
            }
            ImageView connecting_graphic = (ImageView) findViewById(R.id.imageView);
            connecting_graphic.setImageResource(R.drawable.check_mark);
        }
        //else
        else {
            // spawn failure
            ImageView connecting_graphic = (ImageView) findViewById(R.id.imageView);
            connecting_graphic.setImageResource(R.drawable.red_x);
            // spawn back button
            //nextState = new Intent(this, MainActivity.class);
        }
    }


    public class SenderTask extends AsyncTask<String,Void,Boolean> {
        private final ProgressDialog dialog = new ProgressDialog(ServerSender.this);
/*
        protected void onPreExecute() {
            this.dialog.setMessage("Sending...");
            this.dialog.setCancelable(false);
            this.dialog.show();
        }
*/
        protected OutputStream getServerOutput(){
            try {
                return server.getOutputStream();
            } catch (IOException e) {
                e.printStackTrace();
            }
            return null;
        }

        protected InputStream getServerInput(){
            try {
                return server.getInputStream();
            } catch (IOException e) {
                e.printStackTrace();
            }
            return null;
        }

        @Override
        protected Boolean doInBackground(String... strings){
            int i = 0;
            while(!connectToServer(ip,port) && i < 10000){
                i++;
            }
                sendFile(outputFile.getPath());
                return true;
        }

        protected void onPostExecute(Void result) {
            if (dialog.isShowing()) {
                dialog.dismiss();
            }
        }

        public boolean connectToServer(String ip, int port){
            try {
                server= new Socket(ip, port);
                return true;
            } catch(Exception e){
                return false;
            }
        }

        public void sendFile(String path){
            Log.d("ScanActivity", "Starting to send");

            FileInputStream in = null;
            try {
                OutputStream outputStream = server.getOutputStream();
                InputStream inputStream = server.getInputStream();
                Log.d("ScanActivity", "Creating file");
                in = new FileInputStream(path);
                // Write to the stream:
                byte[] buffer = new byte[1024]; // 1KB buffer size
                int length = 0;
                Log.d("ScanActivity", "Begining send");
                while ((length = in.read(buffer, 0, buffer.length)) != -1) {
                    outputStream.write(buffer, 0, length);
                }
                Log.d("ScanActivity", "Sent");
                outputStream.flush();
            }
            catch (Exception e){

            }

        }


    }
}
