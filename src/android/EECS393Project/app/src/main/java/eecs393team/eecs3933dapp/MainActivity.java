package eecs393team.eecs3933dapp;

import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.view.inputmethod.InputMethodManager;
import android.widget.EditText;

public class MainActivity extends AppCompatActivity {

    public static String serverIp = "0.0.0.0";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

    }

    public void create(View view){
        Intent intent = new Intent(this, CreateMain.class);
        startActivity(intent);
    }

    public void scan(View view){
        serverIp = ((EditText)findViewById(R.id.server_ip)).getText().toString();
        Log.d("Main Activity", "IP set");
        Intent intent = new Intent(this, CaptureActivity.class);
        //Intent intent = new Intent(this, VideoCapture.class);
        startActivity(intent);
    }


    public void calibrate(View view){
        Intent intent = new Intent(this, CalibrationActivity.class);
        startActivity(intent);
    }

    public void edit(View view){
        Intent intent = new Intent(this, Gallery.class);
        startActivity(intent);
    }

    public void print(View view){
        Intent intent = new Intent(this, PrintMain.class);
        startActivity(intent);
    }

    public void focusServerIP(MenuItem item){
        EditText field = (EditText) findViewById(R.id.server_ip);
        field.setFocusableInTouchMode(true);
        field.requestFocus();
        InputMethodManager im = (InputMethodManager)getSystemService(Context.INPUT_METHOD_SERVICE);
        im.showSoftInput(field, 0);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }
}
