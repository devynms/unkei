package eecs393team.eecs3933dapp;

import android.app.Application;
import android.app.Instrumentation;
import android.content.Context;
import android.content.Intent;
import android.test.ActivityInstrumentationTestCase2;
import android.test.ActivityUnitTestCase;
import android.test.TouchUtils;
import android.test.UiThreadTest;
import android.test.suitebuilder.annotation.LargeTest;
import android.test.suitebuilder.annotation.MediumTest;
import android.test.suitebuilder.annotation.SmallTest;
import android.widget.Button;
import android.widget.ImageView;

/**
 * Created by austin on 10/24/15.
 */
public class CreateMainTests extends ActivityInstrumentationTestCase2<CreateMain> {

    CreateMain mCreateMain;
    Button launchNextButton;
    ImageView connecting_graphic;

    public CreateMainTests() {
        super(CreateMain.class);
    }

    @Override
    protected void setUp() throws Exception {
        super.setUp();
        mCreateMain= getActivity();
        launchNextButton = (Button) getActivity().findViewById(R.id.button);
        connecting_graphic = (ImageView) getActivity().findViewById(R.id.imageView);
    }

    @SmallTest
    public void testPreconditions() {
        System.out.println("Testing Preconditions...");
        assertNotNull("mCreateMain is null", mCreateMain);
        assertEquals("Activity is of wrong type",
                CreateMain.class, mCreateMain.getClass());
        System.out.println("Done.");
    }

    @LargeTest
    public void testConnectionFailed(){
        mCreateMain.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mCreateMain.setServer("not an ip");
                mCreateMain.connectToServer();
                assertEquals("Bad text", "Back", launchNextButton.getText());
            }
        });
        Instrumentation.ActivityMonitor mainMonitor =
                getInstrumentation().addMonitor(MainActivity.class.getName(),
                        null, false);
        TouchUtils.clickView(this, launchNextButton);
        MainActivity mainActivity = (MainActivity)
                mainMonitor.waitForActivityWithTimeout(1000);
        assertNotNull("MainActivity is null", mainActivity);
        assertEquals("Monitor for MainActivity has not been called", 1, mainMonitor.getHits());
        assertEquals("Activity is of wrong type",
                MainActivity.class, mainActivity.getClass());
        getInstrumentation().removeMonitor(mainMonitor);
        mainActivity.finish();
    }

    @LargeTest
    public void testConnectionSuccessful(){
        mCreateMain.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mCreateMain.setServer("172.19.6.209");
                mCreateMain.connectToServer();
            }
        });
        assertEquals("Bad text", "Start Video", launchNextButton.getText());
    }

    @SmallTest
    public void testGetServer(){
        assertEquals(mCreateMain.getServer().getServerIP(), "172.19.6.209");
    }
}


