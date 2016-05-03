package me.unkei.unkei;


import android.os.Environment;
import android.util.Log;

import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.text.SimpleDateFormat;
import java.util.Date;

public class ServerListener implements Runnable {

    private ServerSocket server = null;
    private Socket connection = null;
    private final int port = 8082;
    private String ip;

    public ServerListener(String ip){
        this.ip = ip;
    }


    public File initializeSTL(){

        File dir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), "STLS");
        if (!dir.exists()) {
            if (!dir.mkdirs())
                Log.d("init STL", "Folder not created");
            else
                Log.d("init STL", "Folder created");
        }
        String path = dir.getAbsolutePath();
        String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
        File stl = new File(path + File.separator + "MESH"+ timeStamp + ".stl");

        return stl;
    }

    @Override
    public void run() {
        try {
            server = new ServerSocket(port);
            connection = server.accept();
            InputStream in = connection.getInputStream();
            if (in == null) {
                Log.d("File Recieve", "No inputstream");
            }

            File stl = initializeSTL();
            FileOutputStream out = new FileOutputStream(stl);
            //BufferedOutputStream bos = new BufferedOutputStream(out);
            DataInputStream dstream = new DataInputStream(in);

            int size = 0;
            while (size == 0) {
                Log.d("File Receive", "still waiting");
                size = dstream.readInt();
            }
            Log.d("File Receive", "buffer size is: " + size);
            byte[] buffer = new byte[1024]; // 1KB buffer size

            int length = dstream.read(buffer);
            while (size > 0) {
                if(length >= 0) {
                    out.write(buffer, 0, length);
                    size -= length;
                }
                length = dstream.read(buffer);
            }

            Log.d("File Receive", "done");
            Log.d("File Receive", "remaining size: " + size);
            out.flush();
            server.close(); // Will close the outputStream, too.

        } catch(IOException io){
            Log.e("Receiver","Can't receive stl file " + io.getMessage());
        }
    }
}
