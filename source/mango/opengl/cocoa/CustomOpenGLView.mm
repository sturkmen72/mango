/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2025 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#ifndef MANGO_OPENGL_CONTEXT_NONE

#include "CustomOpenGLView.h"

namespace
{
    using namespace mango;

    Keycode computeKeyCode(u32 code)
    {
        static
        const Keycode table [] =
        {
            // offset: 0
            KEYCODE_A,
            KEYCODE_S,
            KEYCODE_D,
            KEYCODE_F,
            KEYCODE_H,
            KEYCODE_G,
            KEYCODE_Z,
            KEYCODE_X,
            KEYCODE_C,
            KEYCODE_V,
            // offset: 10
            KEYCODE_NONE, // backtick
            KEYCODE_B,
            KEYCODE_Q,
            KEYCODE_W,
            KEYCODE_E,
            KEYCODE_R,
            KEYCODE_Y,
            KEYCODE_T,
            KEYCODE_1,
            KEYCODE_2,
            // offset: 20
            KEYCODE_3,
            KEYCODE_4,
            KEYCODE_6,
            KEYCODE_5,
            KEYCODE_NONE, // equal sign
            KEYCODE_9,
            KEYCODE_7,
            KEYCODE_NONE, // minus
            KEYCODE_8,
            KEYCODE_0,
            // offset: 30
            KEYCODE_NONE, // right bracket
            KEYCODE_O,
            KEYCODE_U,
            KEYCODE_NONE, // left bracket
            KEYCODE_I,
            KEYCODE_P,
            KEYCODE_RETURN,
            KEYCODE_L,
            KEYCODE_J,
            KEYCODE_NONE, // apostrophe
            // offset: 40
            KEYCODE_K,
            KEYCODE_NONE, // semicolon
            KEYCODE_NONE, // backslash
            KEYCODE_NONE, // comma
            KEYCODE_NONE, // slash
            KEYCODE_N,
            KEYCODE_M,
            KEYCODE_NONE, // period
            KEYCODE_TAB,
            KEYCODE_SPACE,
            // offset: 50
            KEYCODE_NONE,
            KEYCODE_BACKSPACE,
            KEYCODE_NONE,
            KEYCODE_ESC,
            KEYCODE_NONE, // RSUPER,
            KEYCODE_NONE, // LSUPER,
            KEYCODE_LEFT_SHIFT,
            KEYCODE_CAPS_LOCK,
            KEYCODE_LEFT_ALT,
            KEYCODE_LEFT_CONTROL,
            // offset: 60
            KEYCODE_RIGHT_SHIFT,
            KEYCODE_RIGHT_ALT,
            KEYCODE_RIGHT_CONTROL,
            KEYCODE_NONE, // Fn
            KEYCODE_NONE, // F17
            KEYCODE_DECIMAL,
            KEYCODE_NONE,
            KEYCODE_MULTIPLY,
            KEYCODE_NONE,
            KEYCODE_ADDITION,
            // offset: 70
            KEYCODE_NONE,
            KEYCODE_NUMLOCK,
            KEYCODE_NONE, // volume up
            KEYCODE_NONE, // volume down
            KEYCODE_NONE, // mute
            KEYCODE_DIVIDE,
            KEYCODE_ENTER,
            KEYCODE_NONE,
            KEYCODE_SUBTRACT,
            KEYCODE_NONE, // F18
            // offset: 80
            KEYCODE_NONE, // F19
            KEYCODE_NONE, // keypad equal
            KEYCODE_NUMPAD0,
            KEYCODE_NUMPAD1,
            KEYCODE_NUMPAD2,
            KEYCODE_NUMPAD3,
            KEYCODE_NUMPAD4,
            KEYCODE_NUMPAD5,
            KEYCODE_NUMPAD6,
            KEYCODE_NUMPAD7,
            // offset: 90
            KEYCODE_NONE, // F20
            KEYCODE_NUMPAD8,
            KEYCODE_NUMPAD9,
            KEYCODE_NONE,
            KEYCODE_NONE,
            KEYCODE_NONE,
            KEYCODE_F5,
            KEYCODE_F6,
            KEYCODE_F7,
            KEYCODE_F3,
            // offset: 100
            KEYCODE_F8,
            KEYCODE_F9,
            KEYCODE_NONE,
            KEYCODE_F11,
            KEYCODE_NONE,
            KEYCODE_NONE, // F13
            KEYCODE_NONE, // F16
            KEYCODE_NONE, // F14
            KEYCODE_NONE,
            KEYCODE_F10,
            // offset: 110
            KEYCODE_NONE,
            KEYCODE_F12,
            KEYCODE_NONE,
            KEYCODE_NONE, // F15
            KEYCODE_INSERT, // NOTE: only with non-mac keyboards
            KEYCODE_HOME,
            KEYCODE_PAGE_UP,
            KEYCODE_DELETE,
            KEYCODE_F4,
            KEYCODE_END,
            // offset: 120
            KEYCODE_F2,
            KEYCODE_PAGE_DOWN,
            KEYCODE_F1,
            KEYCODE_LEFT,
            KEYCODE_RIGHT,
            KEYCODE_DOWN,
            KEYCODE_UP,
            KEYCODE_NONE,
        };

        if (code >= 128)
        {
            return KEYCODE_NONE;
        }
        
        return table[code];
    }

    u32 computeKeyMask(NSUInteger flags)
    {
        u32 mask = 0;
        if (flags & NSEventModifierFlagControl)    mask |= KEYMASK_CONTROL;
        if (flags & NSEventModifierFlagShift)      mask |= KEYMASK_SHIFT;
        if (flags & NSEventModifierFlagCommand)    mask |= KEYMASK_SUPER;
        return mask;
    }

    class WindowAdapter : public OpenGLContext
    {
    public:
        void setKeyState(Keycode code, bool press)
        {
            const int keyIndex = int(code) >> 5;
            const int keyMask = 1 << (int(code) & 31);

            if (press)
            {
                m_window_context->keystate[keyIndex] |= keyMask;
            }
            else
            {
                m_window_context->keystate[keyIndex] &= ~keyMask;
            }
        }
    };

} // namespace

@implementation CustomView

- (id)initWithFrame:(NSRect)frame andCustomWindow:(mango::OpenGLContext *)theContext
{
    if ((self = [super initWithFrame:frame]))
    {
        context = theContext;
    }

    // enable file drop events
    [self registerForDraggedTypes:[NSArray arrayWithObjects: NSPasteboardTypeFileURL, nil]];
    
    return self;
}

- (void)prepareOpenGL
{
    [super prepareOpenGL];
    [self setWantsBestResolutionOpenGLSurface:YES];
}

- (void)dealloc
{
    [super dealloc];
}

- (BOOL)isOpaque
{
    return YES;
}

- (BOOL)canBecomeKeyView
{
    return YES;
}

- (BOOL)acceptsFirstResponder
{
    return YES;
}

- (void)trackContentView:(NSWindow *)window
{
    self.translatesAutoresizingMaskIntoConstraints = NO;
    
    NSArray *verticalConstraints = [NSLayoutConstraint constraintsWithVisualFormat:@"V:|[subView]|"
                                                                           options:0
                                                                           metrics:nil
                                                                           views:@{@"subView" : self}];
    
    NSArray *horizontalConstraints = [NSLayoutConstraint constraintsWithVisualFormat:@"H:|[subView]|"
                                                                             options:0
                                                                             metrics:nil
                                                                             views:@{@"subView" : self}];

    [[window contentView] addConstraints:verticalConstraints];
    [[window contentView] addConstraints:horizontalConstraints];
}

- (void)dispatchResize:(NSRect)frame
{
    frame = [self convertRectToBacking:frame]; // NOTE: Retina conversion
    context->onResize(frame.size.width, frame.size.height);
}

- (void)keyDown:(NSEvent *)event
{
    if (event.isARepeat != YES)
    {
        u32 mask = computeKeyMask(event.modifierFlags);
        Keycode code = computeKeyCode(event.keyCode);
        context->onKeyPress(code, mask);

        WindowAdapter* adapter = reinterpret_cast<WindowAdapter*>(context);
        adapter->setKeyState(code, true);
    }
}

- (void)keyUp:(NSEvent *)event
{
    Keycode code = computeKeyCode(event.keyCode);
    context->onKeyRelease(code);

    WindowAdapter* adapter = reinterpret_cast<WindowAdapter*>(context);
    adapter->setKeyState(code, false);
}

- (void)flagsChanged:(NSEvent *)event
{
    /*
    NSUInteger newModifiers = [event modifierFlags] & NSDeviceIndependentModifierFlagsMask;
    bool press = (newModifiers > window->modifiers);
    window->modifiers = (u32)newModifiers;

    if (press)
    {
        Keycode code = translateKey(event.keyCode);
        window->onKeyPress(code);
        //e.key.mod = convertModifiers(newModifiers);
    }
    else
    {
        Keycode code = translateKey(event.keyCode);
        window->onKeyRelease(code);
        //e.key.mod = convertModifiers(newModifiers);
    }
    */
}

- (void)dispatchMouseMove:(NSEvent *)event
{
    NSPoint pos = [self convertPoint:[event locationInWindow] fromView:nil];
    NSRect frame = [self frame];

    pos = [self convertPointToBacking:pos]; // retina conversion
    
    int x = int(pos.x);
    int y = int(frame.size.height - pos.y);
    context->onMouseMove(x, y);
}

- (void)mouseMoved:(NSEvent *)event
{
    [self dispatchMouseMove:event];
}

- (void)mouseDragged:(NSEvent *)event
{
    [self dispatchMouseMove:event];
}

- (void)rightMouseDragged:(NSEvent *)event
{
    [self dispatchMouseMove:event];
}

- (void)otherMouseDragged:(NSEvent *)event
{
    [self dispatchMouseMove:event];
}

- (void)dispatchMouseClick:(NSEvent *)event andMouseButton:(mango::MouseButton)button andClickCount:(NSInteger)clickCount
{
    NSPoint pos = [self convertPoint:[event locationInWindow] fromView:nil];
    NSRect frame = [self frame];

    pos = [self convertPointToBacking:pos]; // retina conversion

    int x = int(pos.x);
    int y = int(frame.size.height - pos.y);
    int count = int(clickCount);

    context->onMouseClick(x, y, button, count);
}

- (void)mouseDown:(NSEvent *)event
{
    [self dispatchMouseClick:event andMouseButton:MOUSEBUTTON_LEFT andClickCount:[event clickCount]];
}

- (void)mouseUp:(NSEvent *)event
{
    [self dispatchMouseClick:event andMouseButton:MOUSEBUTTON_LEFT andClickCount:0];
}

- (void)rightMouseDown:(NSEvent *)event
{
    [self dispatchMouseClick:event andMouseButton:MOUSEBUTTON_RIGHT andClickCount:[event clickCount]];
}

- (void)rightMouseUp:(NSEvent *)event
{
    [self dispatchMouseClick:event andMouseButton:MOUSEBUTTON_RIGHT andClickCount:0];
}

- (void)otherMouseDown:(NSEvent *)event
{
    MouseButton button = MouseButton(event.buttonNumber);
    [self dispatchMouseClick:event andMouseButton:button andClickCount:[event clickCount]];
}

- (void)otherMouseUp:(NSEvent *)event
{
    MouseButton button = MouseButton(event.buttonNumber);
    [self dispatchMouseClick:event andMouseButton:button andClickCount:0];
}

- (void)scrollWheel:(NSEvent *)event
{
    [super scrollWheel:event];
    [self dispatchMouseClick:event andMouseButton:MOUSEBUTTON_WHEEL andClickCount:[event deltaY] * 10.0];
}

- (void) mouseEntered:(NSEvent *)event
{
}

- (void) mouseExited:(NSEvent *)event
{
}

- (void)drawRect:(NSRect)dirtyRect
{
    (void)dirtyRect;
    context->onDraw();
}

- (NSDragOperation) draggingEntered:(id<NSDraggingInfo>)sender
{
    if ((NSDragOperationGeneric & [sender draggingSourceOperationMask]) == NSDragOperationGeneric)
        return NSDragOperationGeneric;
    else
        return NSDragOperationNone;
    
}

- (BOOL) prepareForDragOperation:(id<NSDraggingInfo>)sender
{
    return YES;
}

- (BOOL) performDragOperation:(id <NSDraggingInfo>)sender
{
    NSPasteboard *pasteboard = [sender draggingPasteboard];

    NSArray *classes = [NSArray arrayWithObject:[NSURL class]];
    NSDictionary *options = [NSDictionary dictionaryWithObject:
                            [NSNumber numberWithBool:YES] forKey:NSPasteboardURLReadingFileURLsOnlyKey];
    NSArray *fileURLs = [pasteboard readObjectsForClasses:classes options:options];
    
    int numberOfFiles = (int) [fileURLs count];

    if (numberOfFiles > 0)
    {
        filesystem::FileIndex dropped;

        for (int i = 0; i < numberOfFiles; ++i)
        {
            NSString *path = [[fileURLs objectAtIndex:i] path];

            BOOL isDirectory;
            [[NSFileManager defaultManager] fileExistsAtPath:path isDirectory:&isDirectory];

            std::string s = [path UTF8String];
            if (isDirectory)
            {
                dropped.emplace(s + "/", 0, mango::filesystem::FileInfo::DIRECTORY);
            }
            else
            {
                u64 size = [[[NSFileManager defaultManager] attributesOfItemAtPath:path error:nil] fileSize];
                dropped.emplace(s, size, 0);
            }
        }

        context->onDropFiles(dropped);
    }

    return YES;
}

- (void) concludeDragOperation:(id<NSDraggingInfo>)sender
{
    [self setNeedsDisplay:YES];
}

@end

#endif // MANGO_OPENGL_CONTEXT_NONE
