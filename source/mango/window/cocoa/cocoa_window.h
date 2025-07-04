/*
    MANGO Multimedia Development Platform
    Copyright (C) 2012-2025 Twilight Finland 3D Oy Ltd. All rights reserved.
*/
#include <mango/window/window.hpp>

#if defined(MANGO_WINDOW_SYSTEM_COCOA)

#import <Cocoa/Cocoa.h>

namespace mango
{
    struct WindowContext
    {
        // window state
        id      window;
        bool    is_looping;
        u32     keystate[4] = { 0, 0, 0, 0 };
    };

} // namespace mango

// -----------------------------------------------------------------------
// CustomNSWindow
// -----------------------------------------------------------------------

@interface CustomNSWindow : NSWindow {
    mango::Window *window;
}

@property (assign) mango::Window *window;

- (void)createMenu;
@end

#endif // defined(MANGO_WINDOW_SYSTEM_COCOA)
