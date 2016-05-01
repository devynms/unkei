package me.unkei.unkei;


import android.os.Environment;
import android.util.Log;

import java.io.DataInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;
import java.util.concurrent.TimeUnit;

public class ServerListener implements Runnable {

    private InputStream in;
    private Socket server;
    private final int port = 8082;
    private String ip;

    public ServerListener(InputStream in, String ip){
        this.in = in;
        this.server = null;
        this.ip = ip;
    }

    public boolean connectToServer(String ip, int port){
        try {
            server= new Socket(ip, port);
            return true;
        } catch(Exception e){
            return false;
        }
    }

    @Override
    public void run() {
        int i = 0;
        boolean success = false;
        while (!success && i < 10000) {
            success = connectToServer(ip, port);
            i++;
        }
        if (!success) {
            Log.e("Threader", "Failed connection");
            return;
        } else {
            FileOutputStream out = null;
            try {
                this.in = server.getInputStream();
                //InputStream inputStream = new_server.getServerInput();
                if (in == null) {
                    Log.d("File Recieve", "No inputstream");
                    int x = 1 / 0; //I am bad
                }
                Log.d("File Recieve", "obtaining file info");
                File STL = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), "STLS");
                if (!STL.exists()) {
                    if (!STL.mkdirs())
                        Log.d("File Recieve", "Folder not created");
                    else
                        Log.d("File Recieve", "Folder created");
                }
                String path = STL.getAbsolutePath();
                File f = new File(path + "/01.STL");
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
                while (size == 0) {
                    Log.d("File Receive", "still waiting");
                    size = dstream.readInt();
                }
                Log.d("File Receive", "buffer size is: " + size);
                byte[] buffer = new byte[1024]; // 1KB buffer size
                Log.d("File Recieve", "Starting to receive");
                while (size > 0) {
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
                server.close(); // Will close the outputStream, too.
            }
            catch (IOException e) {
                Log.d("File Receive", "Still broke3");
                Log.d("File Receive", e.toString());
                Log.d("File Receive", e.getStackTrace().toString());
            }
        }
    }
}
