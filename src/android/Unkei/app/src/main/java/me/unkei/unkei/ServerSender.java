package me.unkei.unkei;

import android.app.ProgressDialog;
import android.os.AsyncTask;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;

import java.io.FileInputStream;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;

public class ServerSender extends AppCompatActivity {

    private static String ip;
    private Socket server;
    private int port = 8080;
    public String outputPath;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_server_sender);
        Bundle extras = getIntent().getExtras();
        if (extras != null) {
            outputPath = extras.getString("output_path");
        }
        ip = MainActivity.serverIp;
        new SenderTask().execute(outputPath);

    }


    public class SenderTask extends AsyncTask<String,Void,Boolean> {
        private final ProgressDialog dialog = new ProgressDialog(ServerSender.this);

        protected void onPreExecute() {
            this.dialog.setMessage("Sending...");
            this.dialog.setCancelable(false);
            this.dialog.show();
        }

        @Override
        protected Boolean doInBackground(String... strings){
            if(connectToServer(ip,port)){
                sendFile(outputPath);
                return true;
            }
            else{
                return false;
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

        protected void onPostExecute(Void result) {
            if (dialog.isShowing()) {
                dialog.dismiss();
            }
        }
    }
}
