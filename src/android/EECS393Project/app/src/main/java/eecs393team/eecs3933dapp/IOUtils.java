package eecs393team.eecs3933dapp;

import java.io.ByteArrayOutputStream;
import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * Created by Callum on 10/29/2015.
 */
public class IOUtils {
    private static final int DEFAULT_BUFFER_SIZE = 1024 * 4;

    /**
     * Convert <code>input</code> stream into byte[].
     *
     * @param input
     * @return Array of Byte
     * @throws IOException
     */
    public static byte[] toByteArray(InputStream input) throws IOException {
        ByteArrayOutputStream output = new ByteArrayOutputStream();
        copy(input, output);
        return output.toByteArray();
    }

    /**
     * Copy <code>length</code> size of <code>input</code> stream to <code>output</code> stream.
     * This method will NOT close input and output stream.
     *
     * @param input
     * @param output
     * @return long copied length
     * @throws IOException
     */
    private static long copy(InputStream input, OutputStream output) throws IOException {
        byte[] buffer = new byte[DEFAULT_BUFFER_SIZE];
        long count = 0;
        int n = 0;
        while ((n = input.read(buffer)) != -1) {
            output.write(buffer, 0, n);
            count += n;
        }
        return count;
    }


    /**
     * Close <code>closeable</code> quietly.
     *
     * @param closeable
     */
    public static void closeQuietly(Closeable closeable) {
        if (closeable == null) {
            return;
        }

        try {
            closeable.close();
        } catch (Throwable e) {
            // do nothing
        }
    }
}
