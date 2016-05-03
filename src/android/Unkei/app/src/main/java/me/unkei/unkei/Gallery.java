package me.unkei.unkei;

import android.content.Intent;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;

import java.io.File;
import java.util.HashMap;

import android.widget.LinearLayout.LayoutParams;

public class Gallery extends AppCompatActivity {

    private File baseDir;
    public HashMap<String, Button> buttons;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //baseDir = new File("/storage/emulated/legacy/Download");
        buttons = new HashMap<String, Button>();
        setContentView(R.layout.activity_gallery);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        //File dir = getFilesDir();
        File dir = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS), "STLS");
        //dir = baseDir;
        final String path = dir.toString();
        Log.d("Path is:", path);
        Log.d("ExPath is", Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS).toString());
        LinearLayout layout = (LinearLayout)findViewById(R.id.linear);
        LayoutParams params = new LayoutParams(LayoutParams.MATCH_PARENT, LayoutParams.WRAP_CONTENT);
        //make sure the path specified is a valid directory
        if (dir.exists() && dir.isDirectory() && dir.listFiles() != null){
            for (String fileName: dir.list()) {
                Log.d("File is", fileName);
                if (!fileName.endsWith(".stl")){
                    //if the file isn't an STL file, don't allow it to be loaded
                    continue;
                }
                //create button for each STL file
                Button fileButton = new Button(this);
                fileButton.setText(fileName);
                layout.addView(fileButton, params);
                final Gallery currentActivity = this;
                buttons.put(fileName, fileButton);
                fileButton.setOnClickListener(new View.OnClickListener() {
                    public void onClick(View view) {
                        Intent intent = new Intent(currentActivity, STLViewActivity.class);
                        Button b = (Button)view;
                        String filePath = path + File.separator + b.getText().toString();
                        intent.putExtra("file_path", filePath);
                        startActivity(intent);
                    }
                });
            }
        }

        getSupportActionBar().setDisplayHomeAsUpEnabled(true);
    }
}
