package com.wcp.lib.geometry;

import com.wcp.lib.util.CSVWritable;
import com.wcp.lib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    S add(S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
