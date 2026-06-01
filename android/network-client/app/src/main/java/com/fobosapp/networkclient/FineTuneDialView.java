package com.fobosapp.networkclient;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.os.SystemClock;
import android.view.MotionEvent;
import android.view.View;

final class FineTuneDialView extends View {
    interface Listener {
        void onFineTuneDelta(double deltaHz);
    }

    interface ModeListener {
        void onHoldOffsetModeChanged(boolean enabled);
    }

    private static final double MIN_RANGE_HZ = 500.0;
    private static final double MAX_RANGE_HZ = 500_000.0;
    private static final double RANGE_DIVISOR = 20.0;
    private static final long DOUBLE_TAP_MS = 340L;

    private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private Listener listener;
    private ModeListener modeListener;
    private double rangeHz = 10_000.0;
    private double visualOffsetHz = 0.0;
    private float lastX = 0.0f;
    private float tapX = 0.0f;
    private float tapY = 0.0f;
    private long lastTapMs = 0L;
    private boolean dragging = false;
    private boolean holdOffsetMode = false;

    FineTuneDialView(Context context) {
        super(context);
        setMinimumWidth(dp(140));
        setMinimumHeight(dp(44));
    }

    void setListener(Listener nextListener) {
        listener = nextListener;
    }

    void setModeListener(ModeListener nextListener) {
        modeListener = nextListener;
    }

    void setHoldOffsetMode(boolean enabled) {
        if (holdOffsetMode == enabled) {
            return;
        }
        holdOffsetMode = enabled;
        if (!holdOffsetMode) {
            visualOffsetHz = 0.0;
        }
        invalidate();
        if (modeListener != null) {
            modeListener.onHoldOffsetModeChanged(holdOffsetMode);
        }
    }

    boolean isHoldOffsetMode() {
        return holdOffsetMode;
    }

    void setVisibleSpanHz(double visibleSpanHz) {
        if (Double.isNaN(visibleSpanHz) || Double.isInfinite(visibleSpanHz) || visibleSpanHz <= 0.0) {
            return;
        }
        rangeHz = Math.max(MIN_RANGE_HZ, Math.min(MAX_RANGE_HZ, visibleSpanHz / RANGE_DIVISOR));
        visualOffsetHz = Math.max(-rangeHz, Math.min(rangeHz, visualOffsetHz));
        invalidate();
    }

    double getRangeHz() {
        return rangeHz;
    }

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        int preferredWidth = dp(150);
        int preferredHeight = dp(46);
        int width = resolveSize(preferredWidth, widthMeasureSpec);
        int height = resolveSize(preferredHeight, heightMeasureSpec);
        setMeasuredDimension(Math.max(dp(110), width), Math.max(dp(40), height));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        int width = getWidth();
        int height = getHeight();
        float centerX = width * 0.5f;
        double pixelsPerHz = 1.0 / hzPerPixel();
        double majorStepHz = rangeHz / 4.0;
        double minorStepHz = majorStepHz / 5.0;
        int accent = holdOffsetMode ? Color.rgb(242, 91, 91) : Color.rgb(92, 220, 128);

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(Color.rgb(15, 21, 27));
        RectF area = new RectF(1, 1, width - 1, height - 1);
        canvas.drawRoundRect(area, dp(4), dp(4), paint);

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(1));
        paint.setColor(Color.rgb(48, 64, 72));
        canvas.drawRoundRect(area, dp(4), dp(4), paint);

        paint.setTextSize(dp(8));
        paint.setTextAlign(Paint.Align.CENTER);
        for (int i = -20; i <= 20; ++i) {
            double hz = i * minorStepHz + visualOffsetHz;
            float x = (float) (centerX + hz * pixelsPerHz);
            if (x < -20.0f || x > width + 20.0f) {
                continue;
            }
            boolean major = i % 5 == 0;
            paint.setStrokeWidth(major ? dp(1.3f) : dp(1));
            paint.setColor(major ? Color.rgb(105, 130, 140) : Color.rgb(58, 78, 86));
            float tickTop = major ? dp(7) : dp(14);
            float tickBottom = major ? height - dp(13) : height - dp(18);
            canvas.drawLine(x, tickTop, x, tickBottom, paint);
            if (major) {
                paint.setStyle(Paint.Style.FILL);
                paint.setColor(Color.rgb(182, 198, 207));
                canvas.drawText(tickLabel(hz), x, height - dp(4), paint);
                paint.setStyle(Paint.Style.STROKE);
            }
        }

        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(dp(2));
        paint.setColor(accent);
        canvas.drawLine(centerX, dp(3), centerX, height - dp(3), paint);

        paint.setStyle(Paint.Style.FILL);
        paint.setColor(accent);
        canvas.drawCircle(centerX, height * 0.5f, dp(3), paint);

        if (holdOffsetMode || Math.abs(visualOffsetHz) >= 1.0) {
            paint.setTextAlign(Paint.Align.RIGHT);
            paint.setTextSize(dp(8));
            paint.setColor(accent);
            canvas.drawText(offsetLabel(), width - dp(6), dp(14), paint);
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        switch (event.getActionMasked()) {
            case MotionEvent.ACTION_DOWN:
                if (isDoubleTap(event)) {
                    setHoldOffsetMode(!holdOffsetMode);
                    lastTapMs = 0L;
                    return true;
                }
                dragging = true;
                lastX = event.getX();
                if (getParent() != null) {
                    getParent().requestDisallowInterceptTouchEvent(true);
                }
                return true;
            case MotionEvent.ACTION_MOVE:
                if (!dragging) {
                    return true;
                }
                float dx = event.getX() - lastX;
                lastX = event.getX();
                if (Math.abs(dx) < 0.25f) {
                    return true;
                }
                double deltaHz = dx * hzPerPixel();
                visualOffsetHz = Math.max(-rangeHz, Math.min(rangeHz, visualOffsetHz + deltaHz));
                if (listener != null) {
                    listener.onFineTuneDelta(deltaHz);
                }
                invalidate();
                return true;
            case MotionEvent.ACTION_UP:
                rememberTap(event);
                dragging = false;
                if (!holdOffsetMode) {
                    visualOffsetHz = 0.0;
                }
                if (getParent() != null) {
                    getParent().requestDisallowInterceptTouchEvent(false);
                }
                invalidate();
                return true;
            case MotionEvent.ACTION_CANCEL:
                dragging = false;
                if (!holdOffsetMode) {
                    visualOffsetHz = 0.0;
                }
                if (getParent() != null) {
                    getParent().requestDisallowInterceptTouchEvent(false);
                }
                invalidate();
                return true;
            default:
                return true;
        }
    }

    private boolean isDoubleTap(MotionEvent event) {
        long now = SystemClock.uptimeMillis();
        return lastTapMs > 0L &&
                now - lastTapMs <= DOUBLE_TAP_MS &&
                Math.abs(event.getX() - tapX) < dp(24) &&
                Math.abs(event.getY() - tapY) < dp(24);
    }

    private void rememberTap(MotionEvent event) {
        long duration = event.getEventTime() - event.getDownTime();
        if (duration > 260L) {
            return;
        }
        lastTapMs = SystemClock.uptimeMillis();
        tapX = event.getX();
        tapY = event.getY();
    }

    private double hzPerPixel() {
        return rangeHz / Math.max(dp(64), getWidth());
    }

    private String tickLabel(double hz) {
        if (Math.abs(hz) >= 1_000_000.0) {
            return String.format(java.util.Locale.US, "%.1fM", hz / 1_000_000.0);
        }
        if (Math.abs(hz) >= 1_000.0) {
            return String.format(java.util.Locale.US, "%.0fk", hz / 1_000.0);
        }
        return String.format(java.util.Locale.US, "%.0f", hz);
    }

    private String offsetLabel() {
        String sign = visualOffsetHz > 0.0 ? "+" : "";
        if (Math.abs(visualOffsetHz) >= 1_000_000.0) {
            return String.format(java.util.Locale.US, "%s%.2fM", sign, visualOffsetHz / 1_000_000.0);
        }
        if (Math.abs(visualOffsetHz) >= 1_000.0) {
            return String.format(java.util.Locale.US, "%s%.1fk", sign, visualOffsetHz / 1_000.0);
        }
        return String.format(java.util.Locale.US, "%s%.0f", sign, visualOffsetHz);
    }

    private int dp(float value) {
        return (int) (value * getResources().getDisplayMetrics().density + 0.5f);
    }
}
