package me.unkei.unkei;

import android.content.Context;
import android.content.Intent;
import android.os.AsyncTask;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.view.inputmethod.InputMethodManager;
import android.widget.EditText;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;


public class MainActivity extends AppCompatActivity {

    public static String serverIp = "0.0.0.0";

    private static final int PORTNO = 43230;

    private Socket connection;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

    }


    public void scan(View view) {
        serverIp = ((EditText) findViewById(R.id.server_ip)).getText().toString();
        Log.d("Main Activity", "IP set");
        Intent intent = new Intent(this, ScanActivity.class);
        startActivity(intent);
    }

    public void calibrate(View view) {
        Intent intent = new Intent(this, CalibrationActivity.class);
        startActivity(intent);
    }

    public void edit(View view) {
        Intent intent = new Intent(this, Gallery.class);
        startActivity(intent);
    }

    public void userRequest(View view){
        serverIp = ((EditText)findViewById(R.id.server_ip)).getText().toString();
        new UserInfoRequestTask().execute();
    }

    public void focusServerIP(MenuItem item) {
        EditText field = (EditText) findViewById(R.id.server_ip);
        field.setFocusableInTouchMode(true);
        field.requestFocus();
        InputMethodManager im = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE);
        im.showSoftInput(field, 0);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    class UserInfoRequestTask extends AsyncTask<Void, Void, Boolean> {

        public boolean startClient() {

            try {
                connection = new Socket(serverIp, PORTNO);
                return true;
            } catch (Exception e) {
                Log.e("Main", "Failed to connect to server" + e.getMessage());
                return false;
            }
        }

        public void handle() {
            try {
                DataOutputStream toServer = new DataOutputStream(connection.getOutputStream());
                DataInputStream fromServer = new DataInputStream(connection.getInputStream());
                String username = "user";
                String password = "pass";

                toServer.writeShort(username.length());
                toServer.writeShort(password.length());
                toServer.writeShort(2); // cmd 2: request user info
                toServer.writeShort(8); // 64 bit command content
                toServer.writeUTF(username);
                toServer.writeUTF(password);
                toServer.writeLong(0L); // timestamp 0: demand newest version
                toServer.flush();
                int code = fromServer.readInt();
                // 0 for OK. 1 for bad username. 2 for no such resource.
                int dataLen = fromServer.readInt();
                byte[] jsonBytes = new byte[dataLen];
                fromServer.readFully(jsonBytes);
            } catch (IOException io) {
                Log.e("Main", "Failed to connect to server" + io.getMessage());
            }
        }

        @Override
        protected Boolean doInBackground(Void... voids) {
            int i = 0;
            while(!startClient() && i < 10000){
                i++;
            }
            handle();
            return true;
        }
    }
}