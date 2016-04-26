package eecs393team.eecs3933dapp;

import android.os.Environment;
import android.util.Log;

import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.concurrent.TimeUnit;

/**
 * Created by austin on 11/22/15.
 * edited by Josh on 17/04/16
 */
public class ServerListener implements Runnable {

    private InputStream in;
    private ServerConnection server;
    private final int port = 8084;
    private String ip;

    public ServerListener(InputStream in, ServerConnection server, String ip){
        this.in = in;
        this.server = server;
        this.ip = ip;
    }

    @Override
    public void run() {
        ServerConnection new_server = (ServerConnection) new ServerConnection(ip, port).execute();
        this.server = new_server;
        boolean success = false;
        try {
            success = new_server.get(20, TimeUnit.SECONDS);

        } catch(Exception e){

        }
        if (!success){
            Log.d("Threader", "Failed connection");
            return;
        }
        this.in = server.getServerInput();
        FileOutputStream out = null;
        //InputStream inputStream = new_server.getServerInput();
        if (in == null){
            Log.d("File Recieve", "No inputstream");
            int x = 1/0; //I am bad
        }
        try{
            Log.d("File Recieve", "obtaining file info");
            File STL = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), "STLS");
            if (!STL.exists()){
                if (!STL.mkdirs())
                    Log.d("File Recieve", "Folder not created");
                else
                    Log.d("File Recieve", "Folder created");
            }
            String path = STL.getAbsolutePath();
            File f = new File(path+"/01.STL");
            Log.d("File Recieve", "obtaining out info");
            out = new FileOutputStream(f);
            Log.d("File Recieve", "got out info");
            //get buffer size
            //BufferedReader br = new BufferedReader(new InputStreamReader(in, "UTF-8"));
            //Log.d("ScanActivity", "getting size");
            //int bufsize = Integer.parseInt(br.readLine());
            //Log.d("ScanActivity", "got size: " + bufsize);
            DataInputStream dstream = new DataInputStream(in);
            int size = 0;
            while(size == 0){
                Log.d("File Receive", "still waiting");
                size = dstream.readInt();
            }
            Log.d("File Receive", "buffer size is: " + size);
            byte[] buffer = new byte[1024]; // 1KB buffer size
            Log.d("File Recieve", "Starting to receive");
            while(size > 0) {
                int length = dstream.read(buffer);
                out.write(buffer, 0, length);
                size -= length;

            }
            //while ((length = in.read(buffer, 0, buffer.length)) != -1) {
            //    out.write(buffer, 0, length);
            //    Log.d("File Recieve", "Got bytes!");
            //}
            Log.d("File Receive", "done");
            out.flush();
        } catch(Exception e) {
            Log.d("File Receive", "Still broke3");
            Log.d("File Receive", e.toString());
            Log.d("File Receive", e.getStackTrace().toString());
        } finally {
            if (out != null)
                try {
                    out.close();
                }
                catch(Exception e){
                    server.close();
                }
            server.close(); // Will close the outputStream, too.
        }
    }
}
