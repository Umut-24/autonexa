import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';
import 'app_colors.dart';

class AppTheme {
  AppTheme._();

  static ThemeData get dark {
    const c = ResolvedColors.dark();
    return _build(c, Brightness.dark);
  }

  static ThemeData get amoled {
    const c = ResolvedColors.amoled();
    return _build(c, Brightness.dark);
  }

  static ThemeData get light {
    const c = ResolvedColors.light();
    return _build(c, Brightness.light);
  }

  static ThemeData forVariant(ThemeVariant variant) {
    switch (variant) {
      case ThemeVariant.dark: return dark;
      case ThemeVariant.amoled: return amoled;
      case ThemeVariant.light: return light;
    }
  }

  static ThemeData _build(ResolvedColors c, Brightness brightness) {
    final base = brightness == Brightness.dark
        ? ThemeData.dark()
        : ThemeData.light();

    return ThemeData(
      brightness: brightness,
      scaffoldBackgroundColor: c.background,
      colorScheme: ColorScheme(
        brightness: brightness,
        primary: AppColors.brand,
        onPrimary: Colors.white,
        secondary: AppColors.info,
        onSecondary: Colors.white,
        error: AppColors.danger,
        onError: Colors.white,
        surface: c.surface,
        onSurface: c.textPrimary,
      ),
      textTheme: GoogleFonts.interTextTheme(base.textTheme),
      cardTheme: CardThemeData(
        color: c.surface,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(16),
          side: BorderSide(color: c.border, width: 1),
        ),
        elevation: 0,
        margin: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
      ),
      elevatedButtonTheme: ElevatedButtonThemeData(
        style: ElevatedButton.styleFrom(
          backgroundColor: c.isDark ? AppColors.accentDim : AppColors.brand,
          foregroundColor: Colors.white,
          shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
          elevation: 0,
          padding: const EdgeInsets.symmetric(vertical: 14, horizontal: 20),
        ),
      ),
      inputDecorationTheme: InputDecorationTheme(
        filled: true,
        fillColor: c.surfaceLight,
        border: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: BorderSide(color: c.border),
        ),
        enabledBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: BorderSide(color: c.border),
        ),
        focusedBorder: OutlineInputBorder(
          borderRadius: BorderRadius.circular(12),
          borderSide: const BorderSide(color: AppColors.brand, width: 1.5),
        ),
        contentPadding: const EdgeInsets.symmetric(horizontal: 14, vertical: 14),
        hintStyle: TextStyle(color: c.textSecondary, fontSize: 14),
      ),
      bottomSheetTheme: BottomSheetThemeData(
        backgroundColor: c.surface,
        shape: const RoundedRectangleBorder(
          borderRadius: BorderRadius.vertical(top: Radius.circular(20)),
        ),
      ),
      dialogTheme: DialogThemeData(
        backgroundColor: c.surface,
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(20)),
      ),
      snackBarTheme: SnackBarThemeData(
        backgroundColor: c.isDark ? AppColors.darkSurfaceLight : AppColors.lightTextPrimary,
        contentTextStyle: TextStyle(
          color: c.isDark ? AppColors.darkTextPrimary : Colors.white,
        ),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(10)),
        behavior: SnackBarBehavior.floating,
      ),
    );
  }
}
