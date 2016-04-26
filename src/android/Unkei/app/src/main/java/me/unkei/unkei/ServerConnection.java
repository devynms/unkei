package me.unkei.unkei;

import android.os.AsyncTask;

import java.io.IOException;
import java.io.Serializable;
import java.net.Socket;

/**
 * Created by josh on 4/26/16.
 */



public class ServerConnection extends AsyncTask<String, Void, Boolean> implements Serializable {
    private static String ip;
    private Socket server;
    private int port = 8083;

    @Override
    protected Boolean doInBackground(String... strings){
        return connectToServer();
    }

    public boolean connectToServer(){
        try{
            server = new Socket(MainActivity.serverIp,port);
            return true;
        }catch (IOException io){
            return false;
        }
    }
}
